cmake_minimum_required(VERSION 3.10.2)

find_package(gz-cmake3 REQUIRED)

project(gz-match-plugin)


gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})


gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

gz_find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

gz_find_package(gz-msgs10 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})


gz_find_package(sdformat14 REQUIRED)


add_library(
  gz-match-plugin
  SHARED
  src/gz_match_plugin.cpp
)

set_property(TARGET gz-match-plugin PROPERTY cxx_std_17)


#! WARNING ! cppzmq seem to be needed for gz-sim to work, so install via `apt install cppzmq-dev/noble`
# haven't looked into what it does or how to install it automatically as a dependency  
target_link_libraries(gz-match-plugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_include_directories(gz-match-plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



install(TARGETS gz-match-plugin)