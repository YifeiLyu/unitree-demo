cmake_minimum_required(VERSION 3.10)
project(magikid_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# unitree_sdk2 is installed under /usr/local (see install_manifest.txt).
find_package(unitree_sdk2 REQUIRED)

add_executable(magikid_bridge magikid_bridge.cpp)
target_link_libraries(magikid_bridge unitree_sdk2 fmt pthread)
