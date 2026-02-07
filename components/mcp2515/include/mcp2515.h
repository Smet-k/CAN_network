#ifndef MCP2515_H
#define MCP2515_H
#include "driver/gpio.h"
#include "driver/spi_master.h"

typedef struct {
    spi_host_device_t spi_host;
    int clock_hz;
    gpio_num_t gpio_cs;
    gpio_num_t gpio_int;
} mcp2515_config_t;

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t gpio_int;
} mcp2515_handle_t;

typedef enum {
    MCP_OPMODE_NORMAL = 0x00,
    MCP_OPMODE_SLEEP = 0x20,
    MCP_OPMODE_LOOPBACK = 0x40,
    MCP_OPMODE_LISTEN_ONLY = 0x60,
    MCP_OPMODE_CONFIGURATION = 0x80
} mcp2515_opmode_t;

typedef enum {
    MCP_PRIORITY_LOWEST =0x00,
    MCP_PRIORITY_LOW,
    MCP_PRIORITY_HIGH,
    MCP_PRIORITY_HIGHEST
} mcp2515_priority_t;

esp_err_t mcp2515_init(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg);
esp_err_t mcp2515_reset(mcp2515_handle_t* mcp);
esp_err_t mcp2515_read_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t* val);
esp_err_t mcp2515_write_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t val);
esp_err_t mcp2515_wake_up(mcp2515_handle_t* mcp);
esp_err_t mcp2515_set_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t mode);
esp_err_t mcp2515_get_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t* mode);
esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, uint16_t id, uint8_t dlc, const uint8_t* data);

#endif