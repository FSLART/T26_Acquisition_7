# Flashing and debugging through an ST-LINK on SWD.
#
# west flash -> stm32cubeprogrammer: needs STM32_Programmer_CLI on PATH, e.g.
#               C:\ST\STM32CubeCLT_1.23.0\STM32CubeProgrammer\bin
#               Software reset: works even if NRST is not wired to the debug connector.
# west debug -> stlink_gdbserver: ST-LINK_gdbserver from STM32CubeCLT (found in C:\ST
#               automatically, or on PATH) + the Zephyr SDK's GDB.
# west rtt   -> openocd (pass --runner openocd): the Zephyr SDK's OpenOCD with
#               support/openocd.cfg reads the RTT log over SWD through the ST-LINK.
# J-Link stays available with --runner jlink.

board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=sw")
board_runner_args(stlink_gdbserver "--swd")
board_runner_args(jlink "--device=STM32F412RE" "--speed=4000")

# The first include of each kind sets the default: stm32cubeprogrammer for flash,
# stlink_gdbserver for debug
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stlink_gdbserver.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-stm32.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
