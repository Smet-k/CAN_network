#include <math.h>
#include <stdio.h>

#include "bmp280.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"
#include "sdkconfig.h"

#define BMP_SDA_GPIO GPIO_NUM_21
#define BMP_SCL_GPIO GPIO_NUM_22

#define MCP_MISO GPIO_NUM_27
#define MCP_MOSI GPIO_NUM_14
#define MCP_SCK GPIO_NUM_12
#define MCP_CS GPIO_NUM_26
#define MCP_INT GPIO_NUM_13

#define I2C_PORT I2C_NUM_0

#define NETWORK_TEMP_ID     0x100
#define NETWORK_PRESSURE_ID 0x101

void app_main(void) {
    i2c_master_bus_config_t i2c_buscfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = BMP_SDA_GPIO,
        .scl_io_num = BMP_SCL_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .clk_source = I2C_CLK_SRC_DEFAULT};

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_buscfg, &bus_handle));

    bmp280_t bmp280;
    ESP_ERROR_CHECK(bmp280_initialize(&bmp280, BMP_FORCE, bus_handle));

    spi_bus_config_t spi_buscfg = {
        .sclk_io_num = MCP_SCK,
        .mosi_io_num = MCP_MOSI,
        .miso_io_num = MCP_MISO,
        .max_transfer_sz = 128 * 128 * 2};

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_buscfg, SPI_DMA_CH_AUTO));

    mcp2515_handle_t mcp2515;
    mcp2515_config_t mcp_cfg = {
        .spi_host = SPI2_HOST,
        .clock_hz = 1 * 1000 * 1000,
        .gpio_cs = MCP_CS,
        .gpio_int = MCP_INT};

    mcp2515_init(&mcp2515, &mcp_cfg);

    mcp2515_set_opmode(&mcp2515, MCP_OPMODE_LOOPBACK);

    mcp2515_transmit(&mcp2515, NETWORK_TEMP_ID, 25, 5);

}
