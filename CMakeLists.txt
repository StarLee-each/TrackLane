# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(XTENSA_PATH "C:/Users/Administrator/.espressif/tools/xtensa-esp32-elf/esp-2021r2-patch3-8.4.0/xtensa-esp32-elf/bin")
#set(CMAKE_ASM_COMPILER "${XTENSA_PATH}/xtensa-esp32-elf-gcc-8.4.0.exe")
#set(CMAKE_C_COMPILER "${XTENSA_PATH}/xtensa-esp32-elf-gcc.exe")
#set(CMAKE_CXX_COMPILER "${XTENSA_PATH}/xtensa-esp32-elf-g++.exe")
include(C:/Users/Administrator/esp-idf/tools/cmake/project.cmake)

add_compile_definitions(ARDUINO_ARCH_ESP32=1)

# https://cmake.org/cmake/help/latest/envvar/CFLAGS.html
project(track_short)

