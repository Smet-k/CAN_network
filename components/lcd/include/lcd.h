#ifndef LCD_H
#define LCD_H
#include "driver/i2c_master.h"

// driver for LCD1602 display with PCF8574 extender
// LCD runs on HD44780U 

typedef enum {
    LCD_WRAP_NONE = 0x00,
    LCD_WRAP_LINE
} lcd_wrap_mode_t;

typedef struct {
    i2c_master_bus_handle_t bus;

    uint8_t i2c_addr;
    uint8_t rows;
    uint8_t cols;
    lcd_wrap_mode_t word_wrap;
} lcd_config_t;

typedef struct {
    lcd_config_t config;
    bool initialized;
    i2c_master_dev_handle_t i2c;

    uint8_t cursor_col;
    uint8_t cursor_row;
} lcd_handle_t;

typedef enum {
    LCD_LINE_1 = 0x00,
    LCD_LINE_2
} lcd_line_t;
esp_err_t lcd_clear(lcd_handle_t* lcd);
esp_err_t lcd_printf(lcd_handle_t* lcd, const char* fmt, ...);
esp_err_t lcd_print(lcd_handle_t* lcd, char* text);
esp_err_t lcd_set_cursor(lcd_handle_t* lcd, lcd_line_t line, uint8_t offset);
esp_err_t lcd_initialize(lcd_handle_t* lcd, lcd_config_t* cfg);

#endif