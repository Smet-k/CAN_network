idf_component_register(
    SRCS "node_sensor.c"
    INCLUDE_DIRS "."
    REQUIRES bmp280 mcp2515 driver freertos
)
