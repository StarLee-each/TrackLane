# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_definitions(ARDUINO_ARCH_ESP32=1)

# https://cmake.org/cmake/help/latest/envvar/CFLAGS.html
project(track_short)

