/* fw_info block (src/fw_version.c), placed right after the vector table by CMakeLists.txt */
. = ALIGN(16);
KEEP(*(.fw_info))
