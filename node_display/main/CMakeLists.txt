idf_component_register(
    SRCS "node_display.c"
    INCLUDE_DIRS "."
    REQUIRES mcp2515 driver freertos
)
