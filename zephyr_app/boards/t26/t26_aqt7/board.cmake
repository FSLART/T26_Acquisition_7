# Flashing through an ST-LINK on SWD.
# stm32cubeprogrammer (default) needs STM32_Programmer_CLI on PATH, e.g.
# C:\ST\STM32CubeCLT_1.23.0\STM32CubeProgrammer\bin
# Software reset: works even if NRST is not wired to the debug connector.

# keep first
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=sw")
board_runner_args(jlink "--device=STM32F412RE" "--speed=4000")

# keep first
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
