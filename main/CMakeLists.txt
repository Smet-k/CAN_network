file(GLOB SRCS "main.c" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "include"
    REQUIRES bmp280 mcp2515
)