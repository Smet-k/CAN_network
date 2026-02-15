#include <math.h>
#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"
#include "sdkconfig.h"

#define MCP_MISO GPIO_NUM_27
#define MCP_MOSI GPIO_NUM_14
#define MCP_SCK GPIO_NUM_12
#define MCP_CS GPIO_NUM_26
#define MCP_INT GPIO_NUM_13

#define I2C_PORT I2C_NUM_0

#define SPI_BAUD_RATE 4 * 1000 * 1000

#define NETWORK_TEMP_ID 0x100
#define NETWORK_PRESSURE_ID 0x101

#define ID_MASK   0x700
#define ID_FILTER 0x100

void app_main(void) {
    spi_bus_config_t spi_buscfg = {
        .sclk_io_num = MCP_SCK,
        .mosi_io_num = MCP_MOSI,
        .miso_io_num = MCP_MISO,
        .max_transfer_sz = 128 * 128 * 2
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_buscfg, SPI_DMA_CH_AUTO));

    mcp2515_handle_t mcp2515;
    mcp2515_config_t mcp_cfg = {
        .spi_host = SPI2_HOST,
        .clock_hz = SPI_BAUD_RATE,
        .gpio_cs = MCP_CS,
        .gpio_int = MCP_INT,
        .flags.rx_mode = MCP2515_RXM_OFF,
        .flags.RX0_EXIDE = MCP2515_EXIDE_ON
    };
    
    mcp2515_init(&mcp2515, &mcp_cfg);
    mcp2515_configure_timing(&mcp2515, MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_500KBPS);
    mcp2515_set_mask(&mcp2515, MCP2515_MASK0, ID_MASK, true);
    mcp2515_set_filter(&mcp2515, MCP2515_FILTER0, ID_FILTER, true);
    mcp2515_set_opmode(&mcp2515, MCP2515_OPMODE_LOOPBACK);
    
    uint8_t message = 253;
    // make a building helper function?
    mcp2515_frame_t transmit_data = {
        .data = { message },
        .dlc = 1,
        .id = 0xFFF,
        .extended = true
    };
    
    mcp2515_transmit(&mcp2515, &transmit_data);

    mcp2515_frame_t received_data;
    mcp2515_receive(&mcp2515, &received_data);
    printf("0x%lX: %d\n", received_data.id ,*received_data.data);
}
