cmake_minimum_required(VERSION 3.20.0)

# The t26_aqt7 board lives in this folder (boards/t26/t26_aqt7).
# Zephyr doesn't search the application folder for boards on its own, so add it here.
list(APPEND BOARD_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Default board when none is given (west build without -b, CLion "Let CMake decide").
# The board isn't in Zephyr's own board list, so IDE board pickers don't show it.
# A -b / BOARD given on the command line still takes precedence.
set(BOARD t26_aqt7 CACHE STRING "Zephyr board")

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(t26_aqt7)

# Generated DBC code (cantools), updated by a workflow: never edit or move, only reference
set(DBC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Core/DBC)

target_sources(app PRIVATE
  src/main.c
  src/conversions.c
  src/sensors.c
  src/brake_light.c
  src/can_bus.c
  src/can_rx.c
  src/can_tx.c
  src/watchdog.c
  src/debug.c
  ${DBC_DIR}/data_t26.c
  ${DBC_DIR}/autonomous_t26.c
)

# "sim" shell command (CONFIG_AQT7_SIM, on by default when the shell is on)
target_sources_ifdef(CONFIG_AQT7_SIM app PRIVATE src/sim.c)

target_include_directories(app PRIVATE
  ${DBC_DIR}
)
