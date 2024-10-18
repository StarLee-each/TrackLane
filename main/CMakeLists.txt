idf_component_register(
        SRCS
        src/lane_main.cpp
        src/Lane.cpp
        src/LaneCallback.cpp
        src/ScanCallback.cpp
        src/whitelist.cpp
        src/whitelist_esp.cpp
        src/utils.cpp

        INCLUDE_DIRS
        inc
        protocol/inc

        REQUIRES
        spi_flash
        nanopb
        esp_timer
        NeoPixel
        RadioLib
        arduino
        esp-nimble-cpp
        etl
        pthread)

# https://github.com/sheredom/utest.h/issues/19
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++2a -Wformat=0)
