#include "lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#define LCD_ADDR 0x27

#define I2C_TIMEOUT_MS 50

#define LCD_RS          0x01 
#define LCD_RW          0x02
#define LCD_EN          0x04  
#define LCD_BACKLIGHT   0x08

#define CMD_CLEAR_DISPLAY    0x01
#define CMD_RETURN_HOME      0x02
#define CMD_ENTRY_MODE_SET   0x04
#define CMD_DISPLAY_CTRL     0x08
#define CMD_CD_SHIFT         0x10
#define CMD_FUNCTION_SET     0x20
#define CMD_SET_SGRAM_ADDR   0x40
#define CMD_SET_DDRAM_ADDR   0x80

// FUNCTION SET
#define DL_PARAM(bit) (bit << 0x04)
#define N_PARAM(bit)  (bit << 0x03)
#define F_PARAM(bit)  (bit << 0x02)

// DISPLAY CTRL
#define D_PARAM(bit)  (bit << 0x02)
#define C_PARAM(bit)  (bit << 0x01)
#define B_PARAM(bit)  (bit << 0x00)

// ENTRY MODE
#define ID_PARAM(bit) (bit << 0x01)
#define S_PARAM(bit)  (bit << 0x00)

// CURSOR SHIFT
#define SC_PARAM(bit) (bit << 0x03)
#define RL_PARAM(bit) (bit << 0x02)

typedef enum {
    LCD_RS_INSTRUCTION = 0x00,
    LCD_RS_DATA
} lcd_rs_t;

static esp_err_t lcd_write_8_bit(lcd_t* lcd, uint8_t data, lcd_rs_t rs);
static esp_err_t lcd_write_4_bit(lcd_t* lcd, uint8_t data, lcd_rs_t rs);
static esp_err_t lcd_pulse_enable(lcd_t* lcd, uint8_t data);
static esp_err_t lcd_init_DL(lcd_t* lcd);

esp_err_t lcd_initialize(lcd_t* lcd, i2c_master_bus_handle_t bus_handle){
    esp_err_t err;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x27,
        .scl_speed_hz = 100000};
    
    i2c_master_dev_handle_t i2c;

    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c);
    if(err != ESP_OK) return err;

    lcd->i2c = i2c;
    lcd->initialized = false;

    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_init_DL(lcd);

    uint8_t cfg;
    cfg = CMD_FUNCTION_SET | DL_PARAM(0) | N_PARAM(1) | F_PARAM(0);
    err = lcd_write_8_bit(lcd, cfg, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 

    cfg = CMD_DISPLAY_CTRL | D_PARAM(1) | C_PARAM(0) | B_PARAM(0);
    err = lcd_write_8_bit(lcd, cfg, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 

    cfg = CMD_CLEAR_DISPLAY;
    err = lcd_write_8_bit(lcd, cfg, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    cfg = CMD_ENTRY_MODE_SET | ID_PARAM(1) | S_PARAM(0);
    err = lcd_write_8_bit(lcd, cfg, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 

    lcd->initialized = true;

    return ESP_OK;
}

esp_err_t lcd_send_data(lcd_t* lcd, uint8_t data){
    return lcd_write_8_bit(lcd, data, LCD_RS_DATA);
}

static esp_err_t lcd_init_DL(lcd_t* lcd){
    esp_err_t err;

    err = lcd_write_4_bit(lcd, CMD_FUNCTION_SET | DL_PARAM(1), LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(5000);

    err = lcd_write_4_bit(lcd, CMD_FUNCTION_SET | DL_PARAM(1), LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(150);

    err = lcd_write_4_bit(lcd, CMD_FUNCTION_SET | DL_PARAM(1), LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(150);

    err = lcd_write_4_bit(lcd, CMD_FUNCTION_SET | DL_PARAM(0), LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;

    esp_rom_delay_us(150);
    return ESP_OK;
}

static esp_err_t lcd_pulse_enable(lcd_t* lcd, uint8_t data){
    esp_err_t err;

    uint8_t cmd = data | LCD_EN;
    err = i2c_master_transmit(lcd->i2c, &cmd, 1, I2C_TIMEOUT_MS);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(1);

    cmd = data & ~LCD_EN;
    err = i2c_master_transmit(lcd->i2c, &cmd, 1, I2C_TIMEOUT_MS);
    if(err != ESP_OK) return err;

    esp_rom_delay_us(50);

    return ESP_OK;
}

static esp_err_t lcd_write_8_bit(lcd_t* lcd, uint8_t data, lcd_rs_t rs){
    esp_err_t err = lcd_write_4_bit(lcd, data & 0xF0, rs);
    if(err != ESP_OK) return err;

    return lcd_write_4_bit(lcd, (data << 4) & 0xF0, rs);
}

static esp_err_t lcd_write_4_bit(lcd_t* lcd, uint8_t data, lcd_rs_t rs){
    esp_err_t err;
    uint8_t msg = 0;

    msg |= (data & 0xF0);
    if (rs) msg |= LCD_RS;
    msg |= LCD_BACKLIGHT;

    msg &= ~LCD_RW;

    err = i2c_master_transmit(lcd->i2c, &msg, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return err;
    return lcd_pulse_enable(lcd, msg);
}
