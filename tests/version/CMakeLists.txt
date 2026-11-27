# Unit tests for src/fw_version.c (commit hash parsing, fw_info contents), run on the PC with native_sim.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aqt7_version_test)

set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
  src/main.c
  ${APP_SRC}/fw_version.c
)

target_include_directories(app PRIVATE ${APP_SRC})
