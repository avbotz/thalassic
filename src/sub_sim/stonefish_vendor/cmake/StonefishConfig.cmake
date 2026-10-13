# Installed at share/Stonefish/cmake so that find_package(Stonefish) succeeds from the
# workspace prefix alone. The library's own config lives under opt/stonefish_vendor,
# which CMake only searches when an environment hook has added it to CMAKE_PREFIX_PATH.
include("${CMAKE_CURRENT_LIST_DIR}/../../../opt/stonefish_vendor/lib/cmake/Stonefish/StonefishConfig.cmake")
