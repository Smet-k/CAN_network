#include <stdarg.h>
#include "lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#define I2C_TIMEOUT_MS 50

#define LCD_DEFAULT_ROWS 0x02
#define LCD_DEFAULT_COLS 0x10
#define LCD_MAX_ROWS     0x04
#define LCD_MAX_COLS     0x28
#define LCD_LINE_LENGTH  0x40

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

static esp_err_t lcd_write_8_bit(lcd_handle_t* lcd, uint8_t data, lcd_rs_t rs);
static esp_err_t lcd_write_4_bit(lcd_handle_t* lcd, uint8_t data, lcd_rs_t rs);
static esp_err_t lcd_pulse_enable(lcd_handle_t* lcd, uint8_t data);
static esp_err_t lcd_init_DL(lcd_handle_t* lcd);
static esp_err_t lcd_newline(lcd_handle_t* lcd);

static uint8_t lcd_line_addr[2] = {
    0x80,
    0xC0
};

esp_err_t lcd_initialize(lcd_handle_t* lcd, lcd_config_t* cfg){
    if(!lcd || !cfg || !cfg->bus) return ESP_ERR_INVALID_ARG;

    lcd->config = *cfg;

    esp_err_t err;
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = cfg->i2c_addr,
        .scl_speed_hz = 100000};
    
    err = i2c_master_bus_add_device(
        lcd->config.bus,
        &dev_cfg,
        &lcd->i2c   
    );
    if(err != ESP_OK) return err;

    lcd->initialized = false;
    lcd->cursor_col = 1;
    lcd->cursor_row = 1;

    if (lcd->config.cols == 0) lcd->config.cols = LCD_DEFAULT_COLS;
    if (lcd->config.rows == 0) lcd->config.rows = LCD_DEFAULT_ROWS;

    if (lcd->config.cols > LCD_MAX_COLS ||
        lcd->config.rows > LCD_MAX_ROWS)
        return ESP_ERR_INVALID_ARG;

    vTaskDelay(pdMS_TO_TICKS(100));

    lcd_init_DL(lcd);

    uint8_t init_cmd;
    init_cmd = CMD_FUNCTION_SET | DL_PARAM(0) | N_PARAM(1) | F_PARAM(0);
    err = lcd_write_8_bit(lcd, init_cmd, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 

    init_cmd = CMD_DISPLAY_CTRL | D_PARAM(1) | C_PARAM(0) | B_PARAM(0);
    err = lcd_write_8_bit(lcd, init_cmd, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 

    init_cmd = CMD_CLEAR_DISPLAY;
    err = lcd_write_8_bit(lcd, init_cmd, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    init_cmd = CMD_ENTRY_MODE_SET | ID_PARAM(1) | S_PARAM(0);
    err = lcd_write_8_bit(lcd, init_cmd, LCD_RS_INSTRUCTION);
    if(err != ESP_OK) return err;
    esp_rom_delay_us(50); 
    
    lcd->initialized = true;

    return ESP_OK;
}

esp_err_t lcd_printf(lcd_handle_t* lcd, const char* fmt, ...){
    if(!lcd || !fmt) return ESP_ERR_INVALID_ARG;

    char buffer[LCD_MAX_ROWS * LCD_MAX_COLS + 1];

    va_list args;
    va_start(args, fmt);

    int len = vsnprintf(buffer, sizeof(buffer), fmt, args);

    va_end(args);

    if (len < 0)
        return ESP_FAIL;

    if (len >= sizeof(buffer))
        return ESP_ERR_NO_MEM;

    return lcd_print(lcd, buffer);
}

esp_err_t lcd_print(lcd_handle_t* lcd, char* text){
    if(!lcd || !text) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    while(*text){
        if(*text == '\n'){
            err = lcd_newline(lcd);
            if(err != ESP_OK) return err;
            text++;
            continue;
        }

        err = lcd_write_8_bit(lcd, *text++, LCD_RS_DATA);
        if(err != ESP_OK) return err;

        lcd->cursor_col++;
        
        if(lcd->config.word_wrap == LCD_WRAP_LINE && lcd->cursor_col > lcd->config.cols) {
            err = lcd_newline(lcd);
            if(err != ESP_OK) return err;
        }
    }
    return ESP_OK;
}

esp_err_t lcd_set_cursor(lcd_handle_t* lcd, lcd_line_t line, uint8_t offset){
    if(!lcd) return ESP_ERR_INVALID_ARG;

    if(line != LCD_LINE_1 && line != LCD_LINE_2) 
        return ESP_ERR_INVALID_ARG;

    if(offset >= 16)
        return ESP_ERR_INVALID_ARG;
     
    lcd->cursor_row = (line / LCD_LINE_LENGTH) - 1; 
    lcd->cursor_col = offset;

    uint8_t address = lcd_line_addr[line] + offset;   
    return lcd_write_8_bit(lcd, address, LCD_RS_INSTRUCTION);;
}

esp_err_t lcd_clear(lcd_handle_t* lcd){
    if(!lcd) return ESP_ERR_INVALID_ARG;
    esp_err_t err = lcd_write_8_bit(lcd, CMD_CLEAR_DISPLAY, LCD_RS_INSTRUCTION);
    vTaskDelay(pdMS_TO_TICKS(10));
    if(err != ESP_OK) return err;

    lcd->cursor_col = 1;
    lcd->cursor_row = 1;

    return ESP_OK;
}


static esp_err_t lcd_newline(lcd_handle_t* lcd) {
    lcd->cursor_col = 1;
    lcd->cursor_row++;

    if(lcd->cursor_row > lcd->config.rows) return ESP_ERR_INVALID_ARG;
        
    esp_err_t err = lcd_write_8_bit(lcd, lcd_line_addr[lcd->cursor_row - 1], LCD_RS_INSTRUCTION);
    esp_rom_delay_us(50);
    return err;
}

static esp_err_t lcd_init_DL(lcd_handle_t* lcd){
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

static esp_err_t lcd_pulse_enable(lcd_handle_t* lcd, uint8_t data){
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

static esp_err_t lcd_write_8_bit(lcd_handle_t* lcd, uint8_t data, lcd_rs_t rs){
    esp_err_t err = lcd_write_4_bit(lcd, data & 0xF0, rs);
    if(err != ESP_OK) return err;

    return lcd_write_4_bit(lcd, (data << 4) & 0xF0, rs);
}

static esp_err_t lcd_write_4_bit(lcd_handle_t* lcd, uint8_t data, lcd_rs_t rs){
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
