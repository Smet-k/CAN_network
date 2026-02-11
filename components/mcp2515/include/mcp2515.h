#ifndef MCP2515_H
#define MCP2515_H
/**
 * @defgroup mcp2515 MCP2515 Driver
 * @brief Driver for MCP2515 Pressure sensor
 * @{
 */
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define MCP_ROLLOVER_ENABLE 0x01
#define MCP_ROLLOVER_DISABLE 0x00
#define MCP_FILTER_OFF 0x03
#define MCP_FILTER_ON 0x00
typedef struct {
    spi_host_device_t spi_host;
    int clock_hz;
    gpio_num_t gpio_cs;
    gpio_num_t gpio_int;
    struct {
        uint8_t rollover;
        uint8_t rx_mode;
        uint8_t RX_EXIDE;
        uint8_t TX_EXIDE;
    } flags;
} mcp2515_config_t;

typedef struct {
    spi_device_handle_t spi;
    gpio_num_t gpio_int;
} mcp2515_handle_t;

typedef enum {
    MCP2515_CRYSTAL_8MHZ,
    MCP2515_CRYSTAL_16MHZ
} mcp2515_crystal_t;

typedef enum {
    MCP2515_BITRATE_125KBPS,
    MCP2515_BITRATE_250KBPS,
    MCP2515_BITRATE_500KBPS,
    MCP2515_BITRATE_1MBPS
} mcp2515_bitrate_t;

typedef struct{
    mcp2515_crystal_t crystal;
    mcp2515_bitrate_t bitrate;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_timing_cfg_t;

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

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} mcp2515_frame_t;

esp_err_t mcp2515_init(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg);
esp_err_t mcp2515_reset(mcp2515_handle_t* mcp);
esp_err_t mcp2515_read_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t* val);
esp_err_t mcp2515_write_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t val);
esp_err_t mcp2515_wake_up(mcp2515_handle_t* mcp);
esp_err_t mcp2515_set_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t mode);
esp_err_t mcp2515_get_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t* mode);
esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, uint16_t id, uint8_t dlc, const uint8_t* data);
esp_err_t mcp2515_receive(mcp2515_handle_t* mcp, mcp2515_frame_t* frame);
esp_err_t mcp2515_set_filters(mcp2515_handle_t* mcp);
esp_err_t mcp2515_configure_timing(mcp2515_handle_t* mcp, mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate);
/** @} */
#endif