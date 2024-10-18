set(NEOPIXEL_DIR Adafruit_NeoPixel)
set(SRCS ${NEOPIXEL_DIR}/esp.c ${NEOPIXEL_DIR}/Adafruit_NeoPixel.cpp)
idf_component_register(
        SRCS  ${SRCS}
        INCLUDE_DIRS ${NEOPIXEL_DIR}
        REQUIRES arduino
)
