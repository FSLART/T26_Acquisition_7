# Unit tests for src/conversions.c (ADC counts -> bar, mm, C), run on the PC with native_sim.
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aqt7_conversions_test)

set(APP_SRC ${CMAKE_CURRENT_SOURCE_DIR}/../../src)

target_sources(app PRIVATE
  src/main.c
  ${APP_SRC}/conversions.c
)

target_include_directories(app PRIVATE ${APP_SRC})
