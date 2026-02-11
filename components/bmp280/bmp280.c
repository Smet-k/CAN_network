#include "bmp280.h"

#include <math.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define BMP_ADDR 0x76
#define BMP_FORCE_TIMEOUT 100

#define TEMP_CALIBRATION 0x88
#define PRESS_CALIBRATION 0x8E

#define RESET_REG 0xE0
#define STATUS_REG 0xF3
#define CONTROL_REG 0xF4
#define CONFIG_REG 0xF5
#define PRESS_REG 0xF7

#define T_OSRS_X2 0x02
#define P_OSRS_X16 0x05
#define FILTER_X4 0x02

#define CONFIG_T_SB_POS 0x05
#define CONFIG_FILTER_POS 0x02

#define CTRL_OSRS_T_POS 0x05
#define CTRL_OSRS_P_POS 0x02

#define RESET_VALUE 0xB6

#define STATUS_MEASURING_POS 0x03
#define STATUS_MEASURING (1 << STATUS_MEASURING_POS)

/**
 * @brief Read calibration coefficients from BMP280.
 *
 * @param [in,out] bmp280 Initilized device handle.
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_FAIL on invalid calibration data
 *      - ESP_ERR_TIMEOUT on I2C timeout
 *
 * @note Must be called once during initialization.
 */
static esp_err_t bmp280_load_calibration(bmp280_t* bmp280);

/**
 * @brief Compensate raw temperature reading.
 *
 * Implements compensation formula from BMP280 datasheet.
 * Updates t_fine inside device handle.
 *
 * @param[in,out] dev BMP280 device handle.
 * @param[in] raw_temp Raw ADC temperature value.
 *
 * @return Temperature in °C units.
 */
static int32_t bmp280_compensate_t(bmp280_t* dev, const int raw_temp);

/**
 * @brief Compensate raw pressure reading.
 *
 * Requires valid t_fine value from temperature compensation.
 *
 * @param[in] dev BMP280 device handle.
 * @param[in] raw_press Raw ADC pressure value.
 *
 * @return Pressure in Pa.
 *
 * @note bmp280_compensate_t() must be called successfully at least once before this function (to set out the t_fine)
 */
static uint32_t bmp280_compensate_p(bmp280_t* dev, const int32_t raw_press);

esp_err_t bmp280_initialize(bmp280_t* bmp280, bmp280_mode_t work_mode, i2c_master_bus_handle_t bus_handle) {
    esp_err_t err;
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = BMP_ADDR,
        .scl_speed_hz = 100000};

    i2c_master_dev_handle_t i2c;
    err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &i2c);

    bmp280->i2c = i2c;
    bmp280->initialized = false;

    if (err == ESP_OK)
        err = bmp280_load_calibration(bmp280);
    else
        return err;

    bmp280->work_mode = work_mode;

    if (work_mode == BMP_NORMAL) {
        uint8_t tx_buf[2];
        tx_buf[0] = CONFIG_REG;
        tx_buf[1] = (BMP_OSRS_X2 << CONFIG_T_SB_POS) | (BMP_OSRS_X16 << CONFIG_FILTER_POS);

        err = i2c_master_transmit(i2c, tx_buf, sizeof(tx_buf), portMAX_DELAY);
        if (err != ESP_OK) return err;

        tx_buf[0] = CONTROL_REG;
        tx_buf[1] = (BMP_OSRS_X1 << CTRL_OSRS_T_POS) | (BMP_OSRS_X1 << CTRL_OSRS_P_POS) | work_mode;
        err = i2c_master_transmit(i2c, tx_buf, sizeof(tx_buf), portMAX_DELAY);
        if (err != ESP_OK) return err;
    }

    bmp280->initialized = true;
    return err;
}

esp_err_t bmp280_read(bmp280_t* bmp280, bmp280_measurements* measurements) {
    esp_err_t err;
    if (!bmp280->initialized) {
        printf("Measurement error, bmp280 is uninitialized!\n");
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t reg = PRESS_REG;
    uint8_t data[6];

    err = i2c_master_transmit_receive(
        bmp280->i2c, &reg, 1, data, 6, pdMS_TO_TICKS(100));

    if (err != ESP_OK) {
        printf("I2C error: %s\n", esp_err_to_name(err));
        return err;
    }

    int32_t raw_pressure =
        ((int32_t)data[0] << 12) |
        ((int32_t)data[1] << 4) |
        ((int32_t)data[2] >> 4);

    int32_t raw_temperature =
        ((int32_t)data[3] << 12) |
        ((int32_t)data[4] << 4) |
        ((int32_t)data[5] >> 4);

    int32_t temperature = bmp280_compensate_t(bmp280, raw_temperature);
    uint32_t pressure = bmp280_compensate_p(bmp280, raw_pressure);

    measurements->temperature = temperature / 100.0;
    measurements->pressure = pressure / 256.0;

    return err;
}

esp_err_t bmp280_force_measurement(bmp280_t* bmp280, const uint16_t timeout_ms) {
    if (!bmp280 || !bmp280->initialized)
        return ESP_ERR_INVALID_STATE;

    if (bmp280->work_mode != BMP_FORCE)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    uint8_t tx_buf[2] = {
        CONTROL_REG,
        (1 << 5) | (1 << 2) | bmp280->work_mode};

    err = i2c_master_transmit(bmp280->i2c, tx_buf, sizeof(tx_buf), portMAX_DELAY);
    if (err != ESP_OK)
        return err;

    uint8_t status = 0;
    TickType_t start = xTaskGetTickCount();
    do {
        if (xTaskGetTickCount() - start > pdMS_TO_TICKS(timeout_ms))
            return ESP_ERR_TIMEOUT;

        tx_buf[0] = STATUS_REG;
        err = i2c_master_transmit_receive(
            bmp280->i2c, tx_buf, 1, &status, 1, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(5));
    } while (status & STATUS_MEASURING);
    return ESP_OK;
}

esp_err_t bmp280_reset(bmp280_t* bmp280) {
    if (!bmp280)
        return ESP_ERR_INVALID_ARG;

    uint8_t reset_buf[2] = {
        RESET_REG,
        RESET_VALUE};

    esp_err_t err = i2c_master_transmit(bmp280->i2c, reset_buf, sizeof(reset_buf), portMAX_DELAY);

    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(5));

    bmp280->initialized = false;

    return ESP_OK;
}

static esp_err_t bmp280_load_calibration(bmp280_t* bmp280) {
    esp_err_t err;
    uint8_t config_temp = TEMP_CALIBRATION;
    uint8_t buf[6];

    err = i2c_master_transmit_receive(
        bmp280->i2c, &config_temp, 1, buf, 6, pdMS_TO_TICKS(100));

    if (err != ESP_OK)
        return err;

    bmp280->calibration.dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    bmp280->calibration.dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
    bmp280->calibration.dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);

    uint8_t config_pres = PRESS_CALIBRATION;
    uint8_t buf_pres[18];

    err = i2c_master_transmit_receive(
        bmp280->i2c, &config_pres, 1, buf_pres, 18, pdMS_TO_TICKS(100));

    if (err != ESP_OK)
        return err;

    bmp280->calibration.dig_P1 = (uint16_t)(buf_pres[1] << 8 | buf_pres[0]);
    bmp280->calibration.dig_P2 = (int16_t)(buf_pres[3] << 8 | buf_pres[2]);
    bmp280->calibration.dig_P3 = (int16_t)(buf_pres[5] << 8 | buf_pres[4]);
    bmp280->calibration.dig_P4 = (int16_t)(buf_pres[7] << 8 | buf_pres[6]);
    bmp280->calibration.dig_P5 = (int16_t)(buf_pres[9] << 8 | buf_pres[8]);
    bmp280->calibration.dig_P6 = (int16_t)(buf_pres[11] << 8 | buf_pres[10]);
    bmp280->calibration.dig_P7 = (int16_t)(buf_pres[13] << 8 | buf_pres[12]);
    bmp280->calibration.dig_P8 = (int16_t)(buf_pres[15] << 8 | buf_pres[14]);
    bmp280->calibration.dig_P9 = (int16_t)(buf_pres[17] << 8 | buf_pres[16]);

    return err;
}

static int32_t bmp280_compensate_t(bmp280_t* dev, const int raw_temp) {
    int32_t var1, var2, T;
    var1 = ((((raw_temp >> 3) - ((int32_t)dev->calibration.dig_T1 << 1))) *
            ((int32_t)dev->calibration.dig_T2)) >> 11;
    var2 = (((((raw_temp >> 4) - ((int32_t)dev->calibration.dig_T1)) *
              ((raw_temp >> 4) - ((int32_t)dev->calibration.dig_T1))) >> 12) *
            ((int32_t)dev->calibration.dig_T3)) >> 14;

    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t bmp280_compensate_p(bmp280_t* dev, const int32_t raw_press) {
    if (!dev->t_fine) {
        printf("t_fine was not initialized, temperature measurement should be called first.\n");
        return 0;
    }
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->calibration.dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->calibration.dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->calibration.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->calibration.dig_P3) >> 8) +
           ((var1 * (int64_t)dev->calibration.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->calibration.dig_P1) >> 33;
    if (var1 == 0) return 0;
    p = 1048576 - raw_press;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->calibration.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->calibration.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->calibration.dig_P7) << 4);
    return (uint32_t)p;
}