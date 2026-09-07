cmake_minimum_required(VERSION 3.16)
project(stonefish_vendor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

# The Stonefish source is the git submodule next to this file
# (src/sub_sim/stonefish_vendor/stonefish).
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/stonefish/CMakeLists.txt")
  message(FATAL_ERROR
    "Stonefish submodule is missing. Run: git submodule update --init --recursive")
endif()

ament_vendor(stonefish_vendor
  VCS_TYPE path
  VCS_URL stonefish
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=Release
    -DBUILD_TESTS=OFF
  GLOBAL_HOOK
)

ament_package()
