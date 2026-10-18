// magikid_bridge.cpp
// -----------------------------------------------------------------------------
// Magikid Lab - Unitree R1-A telemetry & arm-command bridge.
//
// DDS <-> JSON bridge between unitree_sdk2 (C++/CycloneDDS) and the Python web
// server. It:
//   * subscribes to  rt/lowstate      (all 26 joints + pelvis IMU)  [read-only]
//   * subscribes to  rt/secondary_imu (torso IMU)                   [read-only]
//   * emits one compact JSON state object per tick on STDOUT (newline-delimited)
//   * reads newline-delimited JSON COMMANDS on STDIN.
//
// ARM CONTROL (R1-A upper body): the R1-A is driven at LOW LEVEL over rt/lowcmd
// (12 joints: both arms + head; the arm_sdk blend path needs a high-level
// controller this unit doesn't run). This mirrors the official
// r1A_wrist_swing_example: on engage we best-effort RELEASE the motion-control
// service (so nothing fights us), then publish rt/lowcmd with CRC at 200 Hz.
//
// SAFETY: read-only until an explicit {"type":"arm_engage"}. On engage the arm
// targets are seeded to the CURRENT measured angles (no jump), moves are
// velocity-limited, and every target is clamped to the URDF joint limits. On
// release we restore the motion-control mode that was running before.
// -----------------------------------------------------------------------------

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/IMUState_.hpp>

// Release/restore the robot's motion-control service before low-level control.
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string HG_IMU_TORSO   = "rt/secondary_imu";
static const std::string HG_CMD_TOPIC   = "rt/lowcmd";

// ---- full-body state (26 motors) for the dashboard telemetry --------------
static const int R1_NUM_MOTOR = 26;
static const std::array<int, R1_NUM_MOTOR> IDL_IDX = {
    0, 1, 2, 3, 4, 5,        // left leg
    6, 7, 8, 9, 10, 11,      // right leg
    12, 13,                  // waist roll, waist yaw
    15, 16, 17, 18, 19,      // left arm
    22, 23, 24, 25, 26,      // right arm
    29, 30                   // head pitch, head yaw
};

// ---- low-level arm command set (12 joints: arms + head, no waist) ----------
static const int ARM_N = 12;
// IDL motor indices, matching r1A_wrist_swing_example.
static const std::array<int, ARM_N> ARM_IDL = {
    15, 16, 17, 18, 19,      // left arm  (sh p/r/y, elbow, wrist roll)
    22, 23, 24, 25, 26,      // right arm
    29, 30                   // head pitch, head yaw
};
// State-array indices (0..25) of those same joints, for seeding from lowstate.
static const std::array<int, ARM_N> ARM_STATE_IDX = {
    14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25 };
// Gains from the official R1-A example.
static const std::array<float, ARM_N> ARM_KP = {
    100, 100, 100, 100, 50,  100, 100, 100, 100, 50,  50, 10 };
static const std::array<float, ARM_N> ARM_KD = {
    2, 2, 2, 2, 2,  2, 2, 2, 2, 2,  2, 0.1f };
// URDF joint limits (rad) for clamping targets.
static const std::array<float, ARM_N> ARM_MIN = {
    -3.1416f, -0.22689f, -1.9199f, -0.97564f, -1.9199f,
    -3.1416f, -2.47849f, -1.9199f, -0.97564f, -1.9199f,
    -0.62832f, -2.0071f };
static const std::array<float, ARM_N> ARM_MAX = {
     2.0944f, 2.4784f, 1.9199f, 2.1852f, 1.9199f,
     2.0944f, 0.2268f, 1.9199f, 2.1852f, 1.9199f,
     0.62832f, 2.0071f };

// -----------------------------------------------------------------------------
static uint32_t Crc32Core(uint32_t *ptr, uint32_t len) {
    uint32_t xbit, data, CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1u << 31; data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) { CRC32 <<= 1; CRC32 ^= dwPolynomial; }
            else                    { CRC32 <<= 1; }
            if (data & xbit) CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

// ---- shared robot state ----------------------------------------------------
struct SharedState {
    std::mutex m;
    bool have_state = false, crc_ok = false;
    uint64_t seq = 0;
    uint8_t mode_machine = 0;
    std::array<float, R1_NUM_MOTOR> q{}, dq{}, tau{}, vol{};
    std::array<int,   R1_NUM_MOTOR> temp{}, mstate{};
    std::array<float, 3> imu_rpy{}, imu_gyro{}, imu_acc{};
    std::array<float, 4> imu_quat{};
    bool have_torso = false;
    std::array<float, 3> torso_rpy{};
};
static SharedState g_state;

// ---- arm control state -----------------------------------------------------
struct ArmControl {
    std::mutex m;
    std::atomic<bool> engaged{false};
    std::array<float, ARM_N> q_target{};   // desired angles
    std::array<float, ARM_N> q_cmd{};       // rate-limited command actually sent
    float max_vel = 0.3f;                   // rad/s, velocity limit for moves
    bool have_target = false;
    std::string saved_mode;                 // motion mode to restore on release
    // Scripted motion (home / wave): segments of {duration, 12 targets}, played
    // with cosine easing from wherever the previous segment ended.
    bool traj_active = false;
    std::string traj_name;
    std::vector<float> traj_dur;            // seconds per segment
    std::vector<std::array<float, ARM_N>> traj_q;  // end pose of each segment
    std::array<float, ARM_N> traj_from{};   // start pose of current segment
    size_t traj_seg = 0;
    double traj_t = 0, traj_total = 0, traj_elapsed = 0;
};
// Head yaw is NEVER commanded: its reading is offset by ~100+ deg when the head
// faces forward, and the offset changes between power-ons (seen 2.007, 1.738,
// 1.891 rad all facing forward, 2026-10-04). Commanding it could drive the head
// far sideways. Its slot stays at mode 0 / zero gains (unpowered).
static const int HEAD_YAW_K = 11;                 // index into ARM_* arrays
static const bool HEAD_YAW_ENABLED = false;
// Hard velocity cap while a scripted motion plays (planned peak is lower).
static const float TRAJ_MAX_VEL = 1.0f;
static const int TRAJ_MAX_SEG = 64;
static ArmControl g_arm;

static ChannelPublisherPtr<LowCmd_> g_lowcmd;
static std::unique_ptr<unitree::robot::b2::MotionSwitcherClient> g_msc;

// -----------------------------------------------------------------------------
static void LowStateHandler(const void *message) {
    LowState_ ls = *(const LowState_ *)message;
    bool crc_ok = (ls.crc() == Crc32Core((uint32_t *)&ls, (sizeof(LowState_) >> 2) - 1));
    std::lock_guard<std::mutex> lock(g_state.m);
    g_state.have_state = true; g_state.crc_ok = crc_ok; g_state.seq++;
    g_state.mode_machine = ls.mode_machine();
    auto &ms = ls.motor_state();
    for (int i = 0; i < R1_NUM_MOTOR; ++i) {
        int idx = IDL_IDX[i];
        g_state.q[i]      = ms[idx].q();
        g_state.dq[i]     = ms[idx].dq();
        g_state.tau[i]    = ms[idx].tau_est();
        g_state.temp[i]   = ms[idx].temperature()[0];
        g_state.vol[i]    = ms[idx].vol();
        g_state.mstate[i] = ms[idx].motorstate();
    }
    auto &imu = ls.imu_state();
    for (int k = 0; k < 3; ++k) {
        g_state.imu_rpy[k]  = imu.rpy()[k];
        g_state.imu_gyro[k] = imu.gyroscope()[k];
        g_state.imu_acc[k]  = imu.accelerometer()[k];
    }
    for (int k = 0; k < 4; ++k) g_state.imu_quat[k] = imu.quaternion()[k];
}

static void ImuTorsoHandler(const void *message) {
    IMUState_ imu = *(const IMUState_ *)message;
    std::lock_guard<std::mutex> lock(g_state.m);
    g_state.have_torso = true;
    for (int k = 0; k < 3; ++k) g_state.torso_rpy[k] = imu.rpy()[k];
}

// ---- JSON out --------------------------------------------------------------
static void appendFloatArray(std::ostringstream &os, const char *key, const float *v, int n, int prec = 4) {
    os << '"' << key << "\":[";
    char buf[32];
    for (int i = 0; i < n; ++i) { snprintf(buf, sizeof(buf), "%.*f", prec, v[i]); os << buf; if (i + 1 < n) os << ','; }
    os << ']';
}
static void appendIntArray(std::ostringstream &os, const char *key, const int *v, int n) {
    os << '"' << key << "\":[";
    for (int i = 0; i < n; ++i) { os << v[i]; if (i + 1 < n) os << ','; }
    os << ']';
}
static std::string buildStateJson() {
    SharedState s;
    {
        std::lock_guard<std::mutex> lock(g_state.m);
        s.have_state = g_state.have_state; s.crc_ok = g_state.crc_ok; s.seq = g_state.seq;
        s.mode_machine = g_state.mode_machine;
        s.q = g_state.q; s.dq = g_state.dq; s.tau = g_state.tau;
        s.temp = g_state.temp; s.vol = g_state.vol; s.mstate = g_state.mstate;
        s.imu_rpy = g_state.imu_rpy; s.imu_gyro = g_state.imu_gyro;
        s.imu_acc = g_state.imu_acc; s.imu_quat = g_state.imu_quat;
        s.have_torso = g_state.have_torso; s.torso_rpy = g_state.torso_rpy;
    }
    std::ostringstream os;
    os << '{' << "\"type\":\"state\",";
    os << "\"have_state\":" << (s.have_state ? "true" : "false") << ',';
    os << "\"crc_ok\":" << (s.crc_ok ? "true" : "false") << ',';
    os << "\"seq\":" << s.seq << ',';
    os << "\"mode_machine\":" << int(s.mode_machine) << ',';
    os << "\"arm_engaged\":" << (g_arm.engaged.load() ? "true" : "false") << ',';
    { std::lock_guard<std::mutex> l(g_arm.m);
      os << "\"arm_speed\":" << g_arm.max_vel << ',';
      os << "\"traj_active\":" << (g_arm.traj_active ? "true" : "false") << ',';
      os << "\"traj_name\":\"" << g_arm.traj_name << "\",";
      char b[32]; snprintf(b, sizeof(b), "%.3f", g_arm.traj_total > 0 ? g_arm.traj_elapsed / g_arm.traj_total : 0.0);
      os << "\"traj_prog\":" << b << ','; }
    os << "\"head_yaw_enabled\":" << (HEAD_YAW_ENABLED ? "true" : "false") << ',';
    appendFloatArray(os, "q", s.q.data(), R1_NUM_MOTOR); os << ',';
    appendFloatArray(os, "dq", s.dq.data(), R1_NUM_MOTOR); os << ',';
    appendFloatArray(os, "tau", s.tau.data(), R1_NUM_MOTOR, 2); os << ',';
    appendIntArray(os, "temp", s.temp.data(), R1_NUM_MOTOR); os << ',';
    appendFloatArray(os, "vol", s.vol.data(), R1_NUM_MOTOR, 2); os << ',';
    appendIntArray(os, "mstate", s.mstate.data(), R1_NUM_MOTOR); os << ',';
    appendFloatArray(os, "imu_rpy", s.imu_rpy.data(), 3); os << ',';
    appendFloatArray(os, "imu_gyro", s.imu_gyro.data(), 3); os << ',';
    appendFloatArray(os, "imu_acc", s.imu_acc.data(), 3); os << ',';
    appendFloatArray(os, "imu_quat", s.imu_quat.data(), 4); os << ',';
    os << "\"have_torso\":" << (s.have_torso ? "true" : "false") << ',';
    appendFloatArray(os, "torso_rpy", s.torso_rpy.data(), 3);
    os << '}';
    return os.str();
}

static void logJson(const std::string &msg) {
    // one write per line so it can't interleave with the state stream
    std::cout << ("{\"type\":\"log\",\"msg\":\"" + msg + "\"}\n") << std::flush;
}

// ---- tiny JSON scrapers (fixed schema) -------------------------------------
static bool jsonFindString(const std::string &s, const char *key, std::string &out) {
    std::string k = std::string("\"") + key + "\"";
    auto p = s.find(k); if (p == std::string::npos) return false;
    p = s.find(':', p); if (p == std::string::npos) return false;
    auto q1 = s.find('"', p); if (q1 == std::string::npos) return false;
    auto q2 = s.find('"', q1 + 1); if (q2 == std::string::npos) return false;
    out = s.substr(q1 + 1, q2 - q1 - 1); return true;
}
static bool jsonFindNumber(const std::string &s, const char *key, double &out) {
    std::string k = std::string("\"") + key + "\"";
    auto p = s.find(k); if (p == std::string::npos) return false;
    p = s.find(':', p); if (p == std::string::npos) return false;
    out = strtod(s.c_str() + p + 1, nullptr); return true;
}
static bool jsonFindFloatArray(const std::string &s, const char *key, std::vector<float> &out) {
    std::string k = std::string("\"") + key + "\"";
    auto p = s.find(k); if (p == std::string::npos) return false;
    auto lb = s.find('[', p); if (lb == std::string::npos) return false;
    auto rb = s.find(']', lb); if (rb == std::string::npos) return false;
    out.clear();
    const char *c = s.c_str() + lb + 1, *end = s.c_str() + rb;
    while (c < end) {
        char *nx = nullptr; double v = strtod(c, &nx);
        if (nx == c) break;
        out.push_back((float)v); c = nx;
        while (c < end && (*c == ',' || *c == ' ')) c++;
    }
    return true;
}

// ---- arm control -----------------------------------------------------------
// Caller holds g_arm.m. Stops any scripted motion; q_target stays where it was.
static void trajStopLocked() {
    g_arm.traj_active = false;
    g_arm.traj_dur.clear(); g_arm.traj_q.clear();
    g_arm.traj_seg = 0; g_arm.traj_t = 0; g_arm.traj_elapsed = 0; g_arm.traj_total = 0;
}

// Take low-level control: best-effort release the motion-control service, then
// seed targets to the current measured pose so nothing jumps.
static void armEngage() {
    if (g_arm.engaged.load()) return;
    {
        std::lock_guard<std::mutex> ls(g_state.m);
        if (!g_state.have_state) { logJson("cannot engage: no lowstate yet"); return; }
    }
    if (g_msc) {
        std::string form, name;
        if (g_msc->CheckMode(form, name) == 0 && !name.empty()) {
            g_arm.saved_mode = name;
            for (int k = 0; k < 3 && !name.empty(); ++k) {
                g_msc->ReleaseMode();
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                g_msc->CheckMode(form, name);
            }
        }
    }
    {
        std::lock_guard<std::mutex> l(g_arm.m);
        std::lock_guard<std::mutex> ls(g_state.m);
        for (int i = 0; i < ARM_N; ++i)
            g_arm.q_cmd[i] = g_arm.q_target[i] = g_state.q[ARM_STATE_IDX[i]];
        g_arm.have_target = true;
        trajStopLocked();
    }
    g_arm.engaged.store(true);
    logJson(HEAD_YAW_ENABLED ? "arm control ENGAGED (low-level rt/lowcmd, holding current pose)"
                             : "arm control ENGAGED (holding current pose; head yaw disabled, unpowered)");
}

static void armRelease() {
    { std::lock_guard<std::mutex> l(g_arm.m); trajStopLocked(); }
    if (!g_arm.engaged.load()) return;
    g_arm.engaged.store(false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::string restored;
    if (g_msc && !g_arm.saved_mode.empty()) {
        g_msc->SelectMode(g_arm.saved_mode);
        restored = " (restored mode " + g_arm.saved_mode + ")";
    }
    logJson("arm control released" + restored);
}

static void handleCommand(const std::string &line) {
    std::string type;
    if (!jsonFindString(line, "type", type)) return;

    if (type == "arm_engage") {
        armEngage();
    } else if (type == "arm_release" || type == "estop") {
        armRelease();
    } else if (type == "arm_speed" || type == "arm_weight") {
        double v; if (jsonFindNumber(line, "value", v)) {
            std::lock_guard<std::mutex> l(g_arm.m);
            g_arm.max_vel = (float)std::max(0.02, std::min(1.5, v));
        }
    } else if (type == "arm_set") {
        std::vector<float> q;
        if (jsonFindFloatArray(line, "q", q) && (int)q.size() == ARM_N) {
            { std::lock_guard<std::mutex> l(g_arm.m);
              if (g_arm.traj_active) { trajStopLocked(); logJson("motion interrupted by manual targets"); }
              for (int i = 0; i < ARM_N; ++i)
                  g_arm.q_target[i] = std::max(ARM_MIN[i], std::min(ARM_MAX[i], q[i]));
              g_arm.have_target = true; }
            char buf[96]; snprintf(buf, sizeof(buf),
                "targets set (12 joints); engaged=%d speed=%.2f rad/s%s",
                (int)g_arm.engaged.load(), g_arm.max_vel,
                g_arm.engaged.load() ? "" : " -> engage first to move");
            logJson(buf);
        } else {
            logJson("arm_set ignored (expected 12 joint values)");
        }
    } else if (type == "traj") {
        // {"type":"traj","name":"wave_r","dur":[s,...],"q":[12*N]}
        std::string name; jsonFindString(line, "name", name);
        std::vector<float> dur, q;
        if (!g_arm.engaged.load()) { logJson("motion '" + name + "' rejected: engage first"); return; }
        if (!jsonFindFloatArray(line, "dur", dur) || !jsonFindFloatArray(line, "q", q) ||
            dur.empty() || (int)dur.size() > TRAJ_MAX_SEG || q.size() != dur.size() * ARM_N) {
            logJson("motion rejected: bad trajectory"); return;
        }
        std::lock_guard<std::mutex> l(g_arm.m);
        trajStopLocked();
        g_arm.traj_name = name;
        for (size_t s = 0; s < dur.size(); ++s) {
            std::array<float, ARM_N> p;
            for (int i = 0; i < ARM_N; ++i)
                p[i] = std::max(ARM_MIN[i], std::min(ARM_MAX[i], q[s * ARM_N + i]));
            g_arm.traj_q.push_back(p);
            float d = std::max(0.2f, std::min(30.0f, dur[s]));
            g_arm.traj_dur.push_back(d);
            g_arm.traj_total += d;
        }
        g_arm.traj_from = g_arm.q_cmd;      // start from what we're commanding now
        g_arm.traj_active = true;
        char buf[96]; snprintf(buf, sizeof(buf), "motion '%s' started (%d segments, %.1f s)",
                               name.c_str(), (int)dur.size(), g_arm.traj_total);
        logJson(buf);
    } else if (type == "traj_stop") {
        std::lock_guard<std::mutex> l(g_arm.m);
        if (g_arm.traj_active) {
            trajStopLocked();
            g_arm.q_target = g_arm.q_cmd;   // hold where we are
            logJson("motion stopped, holding pose");
        }
    }
}

// Advance the scripted motion by dt; caller holds g_arm.m.
static void trajStepLocked(double dt) {
    if (!g_arm.traj_active) return;
    g_arm.traj_t += dt; g_arm.traj_elapsed += dt;
    while (g_arm.traj_seg < g_arm.traj_dur.size() && g_arm.traj_t >= g_arm.traj_dur[g_arm.traj_seg]) {
        g_arm.traj_t -= g_arm.traj_dur[g_arm.traj_seg];
        g_arm.traj_from = g_arm.traj_q[g_arm.traj_seg];
        g_arm.traj_seg++;
    }
    if (g_arm.traj_seg >= g_arm.traj_dur.size()) {
        g_arm.q_target = g_arm.traj_q.back();
        std::string name = g_arm.traj_name;
        trajStopLocked();
        logJson("motion '" + name + "' done, holding final pose");
        return;
    }
    double s = g_arm.traj_t / g_arm.traj_dur[g_arm.traj_seg];
    double e = 0.5 - 0.5 * std::cos(M_PI * s);           // cosine ease in/out
    const auto &to = g_arm.traj_q[g_arm.traj_seg];
    for (int i = 0; i < ARM_N; ++i)
        g_arm.q_target[i] = (float)(g_arm.traj_from[i] + (to[i] - g_arm.traj_from[i]) * e);
}

static void stdinThread() {
    std::string line;
    while (std::getline(std::cin, line)) if (!line.empty()) handleCommand(line);
}

// Publish rt/lowcmd at 200 Hz while engaged, velocity-limiting toward target.
static void armControlThread() {
    const double dt = 0.005;
    const float step_default = 0.3f;
    while (true) {
        if (g_arm.engaged.load() && g_lowcmd) {
            LowCmd_ cmd;
            cmd.mode_pr() = 0;
            { std::lock_guard<std::mutex> ls(g_state.m); cmd.mode_machine() = g_state.mode_machine; }
            {
                std::lock_guard<std::mutex> l(g_arm.m);
                trajStepLocked(dt);
                float step = (g_arm.traj_active ? TRAJ_MAX_VEL
                              : (g_arm.max_vel > 0 ? g_arm.max_vel : step_default)) * (float)dt;
                for (int i = 0; i < ARM_N; ++i) {
                    // head yaw disabled: leave the slot at mode 0 / zero gains
                    if (!HEAD_YAW_ENABLED && i == HEAD_YAW_K) continue;
                    float d = g_arm.q_target[i] - g_arm.q_cmd[i];
                    if (d >  step) d =  step;
                    if (d < -step) d = -step;
                    g_arm.q_cmd[i] += d;
                    int idl = ARM_IDL[i];
                    cmd.motor_cmd().at(idl).mode(1);
                    cmd.motor_cmd().at(idl).q(g_arm.q_cmd[i]);
                    cmd.motor_cmd().at(idl).dq(0);
                    cmd.motor_cmd().at(idl).kp(ARM_KP[i]);
                    cmd.motor_cmd().at(idl).kd(ARM_KD[i]);
                    cmd.motor_cmd().at(idl).tau(0);
                }
            }
            cmd.crc() = Crc32Core((uint32_t *)&cmd, (sizeof(LowCmd_) >> 2) - 1);
            g_lowcmd->Write(cmd);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main(int argc, char const *argv[]) {
    if (argc < 2) { std::cerr << "Usage: " << argv[0] << " <network_interface> [rate_hz]\n"; return 1; }
    std::string iface = argv[1];
    int rate_hz = (argc >= 3) ? std::max(1, atoi(argv[2])) : 50;

    ChannelFactory::Instance()->Init(0, iface);

    // Motion-switcher client for releasing/restoring the holding controller.
    try {
        g_msc = std::make_unique<unitree::robot::b2::MotionSwitcherClient>();
        g_msc->SetTimeout(5.0f);
        g_msc->Init();
    } catch (...) { logJson("motion switcher init failed"); }

    ChannelSubscriberPtr<LowState_> lowstate_sub(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_sub->InitChannel(LowStateHandler, 1);
    ChannelSubscriberPtr<IMUState_> imu_sub(new ChannelSubscriber<IMUState_>(HG_IMU_TORSO));
    imu_sub->InitChannel(ImuTorsoHandler, 1);

    g_lowcmd.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    g_lowcmd->InitChannel();

    std::thread(stdinThread).detach();
    std::thread(armControlThread).detach();

    logJson(std::string("bridge up on ") + iface + " (read-only until engaged; R1-A low-level arm control)");

    const auto period = std::chrono::milliseconds(1000 / rate_hz);
    while (true) {
        std::cout << buildStateJson() << '\n' << std::flush;
        std::this_thread::sleep_for(period);
    }
    return 0;
}
