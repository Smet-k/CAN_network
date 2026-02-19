#ifndef LCD_H
#define LCD_H
#include "driver/i2c_master.h"

// driver for LCD1602 display with PCF8574 extender
// LCD runs on HD44780U 

typedef struct {
    i2c_master_dev_handle_t i2c;
    bool initialized;
} lcd_t;

esp_err_t lcd_send_data(lcd_t* lcd, uint8_t data);
esp_err_t lcd_initialize(lcd_t* lcd, i2c_master_bus_handle_t bus_handle);

#endif