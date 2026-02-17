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

#define MCP_MISO GPIO_NUM_16
#define MCP_MOSI GPIO_NUM_17
#define MCP_SCK GPIO_NUM_18
#define MCP_CS GPIO_NUM_4
#define MCP_INT GPIO_NUM_19

#define I2C_PORT I2C_NUM_0

#define SPI_BAUD_RATE 4 * 500 * 1000

#define NETWORK_TEMP_ID 0x100
#define NETWORK_PRESSURE_ID 0x101

#define ID_MASK   0x700
#define ID_FILTER 0x100

typedef union {
    double d;
    uint8_t bytes[8];
} double_bytes_t;

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
        .flags.rx_mode = MCP2515_RXM_ON,
        .flags.RX0_EXIDE = MCP2515_EXIDE_OFF
    };
    
    mcp2515_init(&mcp2515, &mcp_cfg);
    mcp2515_configure_timing(&mcp2515, MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_125KBPS);
    mcp2515_set_opmode(&mcp2515, MCP2515_OPMODE_NORMAL);
    double temperature = 0, pressure = 0;

    while(1){
        mcp2515_frame_t received_data;
         
        if(!gpio_get_level(MCP_INT)){
            ESP_ERROR_CHECK(mcp2515_receive(&mcp2515, &received_data));

            double_bytes_t data;
            for(int i = 0;i < received_data.dlc; i++){
                data.bytes[i] = received_data.data[i];
            }

            switch (received_data.id)
            {
            case NETWORK_TEMP_ID:
                temperature = data.d;
                break;
            case NETWORK_PRESSURE_ID:
                pressure = data.d;
                break;
            default: break;
            }
        }
        // To be replaced with LCD display when i develop the actual driver
        printf("Temperature: %.02f C\n", temperature);
        printf("Pressure: %.02f Pa\n", pressure);
        vTaskDelay(10);
    }
}
