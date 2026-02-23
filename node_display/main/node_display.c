#include <stdio.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcp2515.h"
#include "lcd.h"
#include "sdkconfig.h"

#define MCP_MISO GPIO_NUM_16
#define MCP_MOSI GPIO_NUM_17
#define MCP_SCK GPIO_NUM_18
#define MCP_CS GPIO_NUM_4
#define MCP_INT GPIO_NUM_19

#define LCD_ADDR 0x27
#define LCD_WRITE_ADDR 0x4E
#define LCD_READ_ADDR 0x4F
#define LCD_SDA GPIO_NUM_27
#define LCD_SCL GPIO_NUM_26

#define I2C_PORT I2C_NUM_0

#define SPI_BAUD_RATE 4 * 500 * 1000

#define NETWORK_TEMP_ID 0x100
#define NETWORK_PRESSURE_ID 0x101

#define ID_MASK 0x700
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
        .max_transfer_sz = 128 * 128 * 2};

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &spi_buscfg, SPI_DMA_CH_AUTO));

    i2c_master_bus_config_t i2c_buscfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = LCD_SDA,
        .scl_io_num = LCD_SCL,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
        .clk_source = I2C_CLK_SRC_DEFAULT
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_buscfg, &bus_handle));

    mcp2515_handle_t mcp2515;
    mcp2515_config_t mcp_cfg = {
        .spi_host = SPI2_HOST,
        .clock_hz = SPI_BAUD_RATE,
        .gpio_cs = MCP_CS,
        .gpio_int = MCP_INT,
        .flags.rx_mode = MCP2515_RXM_ON,
        .flags.RX0_EXIDE = MCP2515_EXIDE_OFF};

    ESP_ERROR_CHECK(mcp2515_init(&mcp2515, &mcp_cfg));
    ESP_ERROR_CHECK(mcp2515_configure_timing(&mcp2515, MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_125KBPS));
    ESP_ERROR_CHECK(mcp2515_set_opmode(&mcp2515, MCP2515_OPMODE_NORMAL));

    lcd_config_t lcd_cfg = {
        .bus = bus_handle,
        .i2c_addr = LCD_ADDR,
        .word_wrap = LCD_WRAP_LINE,
        .line_cfg = LCD_LINE_2
    };
    lcd_handle_t lcd;
    ESP_ERROR_CHECK(lcd_initialize(&lcd, &lcd_cfg));

    ESP_ERROR_CHECK(lcd_printf(&lcd, "Temp: 0.00 C\nPres: 0.00 KPa"));
    double temperature = 0, old_temperature = 0, pressure = 0, old_pressure = 0;

    while (1) {
        mcp2515_frame_t received_data;

        if (!gpio_get_level(MCP_INT)) {
            ESP_ERROR_CHECK(mcp2515_receive(&mcp2515, &received_data));

            double_bytes_t data;
            for (int i = 0; i < received_data.dlc; i++) {
                data.bytes[i] = received_data.data[i];
            }

            switch (received_data.id) {
                case NETWORK_TEMP_ID:
                    temperature = data.d;
                    break;
                case NETWORK_PRESSURE_ID:
                    pressure = data.d / 1000.0;
                    break;
                default:
                    break;
            }
        }

        if(temperature != old_temperature){
            ESP_ERROR_CHECK(lcd_set_cursor(&lcd, LCD_LINE_1, 0));
            ESP_ERROR_CHECK(lcd_printf(&lcd, "Temp: %.2f C\n", temperature));
            old_temperature = temperature;
        }

        if(pressure != old_pressure){
            ESP_ERROR_CHECK(lcd_set_cursor(&lcd, LCD_LINE_2, 0));
            ESP_ERROR_CHECK(lcd_printf(&lcd, "Pres: %.2f kPa", pressure));
            old_pressure = pressure;
        }
    }
}
