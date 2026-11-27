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

# Firmware version: Zephyr generates zephyr/app_version.h from the VERSION file (semver, bumped by CI)
# and puts this repository's commit in APP_BUILD_VERSION (git describe --abbrev=12 --always).
# By default it only regenerates when VERSION changes: also regenerate it when a commit, checkout or
# git add changes HEAD, the branch ref or the index, so an incremental build reports the right commit.
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --path-format=absolute
            --git-path HEAD --git-path index --symbolic-full-name HEAD
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE git_out OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE git_result ERROR_QUIET
  )
  if(git_result EQUAL 0)
    string(REPLACE "\n" ";" git_out "${git_out}")
    list(GET git_out 0 git_head)
    list(GET git_out 1 git_index)
    set(version_depends ${git_head} ${git_index})
    # Loose ref of the current branch (absent when packed or on a detached HEAD)
    list(LENGTH git_out git_out_len)
    if(git_out_len GREATER 2)
      list(GET git_out 2 git_branch_ref)
      cmake_path(GET git_head PARENT_PATH git_dir)
      if(EXISTS ${git_dir}/${git_branch_ref})
        list(APPEND version_depends ${git_dir}/${git_branch_ref})
      endif()
    endif()
    set(existing_depends)
    foreach(f IN LISTS version_depends)
      if(EXISTS ${f})
        list(APPEND existing_depends ${f})
      endif()
    endforeach()
    set_property(TARGET app_version_h PROPERTY APP_VERSION_DEPENDS ${existing_depends})
    # A checkout to another branch changes which ref file to watch: reconfigure then
    if(EXISTS ${git_head})
      set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${git_head})
    endif()
  endif()
endif()

# fw_info block (src/fw_version.c) right after the vector table, where a bootloader or the CAN
# updater can find it. The sort key sorts after Zephyr's "0x1vectors" (vector table padding).
zephyr_linker_sources(ROM_START SORT_KEY 0x2fw_info src/fw_info.ld)

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
  src/fw_version.c
  ${DBC_DIR}/data_t26.c
  ${DBC_DIR}/autonomous_t26.c
)

# "sim" shell command (CONFIG_AQT7_SIM, on by default when the shell is on)
target_sources_ifdef(CONFIG_AQT7_SIM app PRIVATE src/sim.c)

target_include_directories(app PRIVATE
  ${DBC_DIR}
)
