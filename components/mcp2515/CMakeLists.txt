file(GLOB SRCS "mcp2515.c" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    REQUIRES driver freertos
)