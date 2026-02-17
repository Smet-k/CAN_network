#ifndef BMP280_H
#define BMP280_H
/**
 * @defgroup bmp280 BMP280 Driver
 * @brief Driver for Bosch BMP280 Pressure sensor
 * @{
 */
#include <stdbool.h>

#include "driver/i2c_master.h"
/**
 * @brief bmp280 working mode (matches datasheet CTRL_MEAS mode bits).
 */
typedef enum {
    BMP_SLEEP = 0x00,
    BMP_FORCE = 0x01,
    BMP_NORMAL = 0x03
} bmp280_mode_t;

/**
 * @brief bmp280 setting values for oversampling
 */
typedef enum {
    BMP_OSRS_SKIP = 0x00,
    BMP_OSRS_X1,
    BMP_OSRS_X2,
    BMP_OSRS_X4,
    BMP_OSRS_X8,
    BMP_OSRS_X16
} bmp280_osrs_t;

/**
 * @brief bmp280 calibration for compensation calculation.
 *
 * Calibration parameters read from the BMP280's non-volatile memory.
 * These values are used by the temperature and pressure compensation
 * functions.
 */
typedef struct {
    /* Temperature calibration */
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;

    /* Pressure calibration*/
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calibration;

/**
 * @brief measurements taken by bmp280.
 */
typedef struct {
    double temperature; /**< Temperature in °C */
    double pressure;    /**< Pressure in Pa */
} bmp280_measurements;

/**
 * @brief bmp280 handler.
 */
typedef struct {
    i2c_master_dev_handle_t i2c;
    int32_t t_fine;

    bmp280_calibration calibration;
    bmp280_mode_t work_mode;

    bool initialized; /**< Set true after successful initialization */
} bmp280_t;

/**
 * @brief Initialize bmp280 device.
 *
 * The bmp280 handle must remain valid for the lifetime of the device usage.
 * @param[out] bmp280 pointer to bmp280 handle.
 * @param[in] work_mode bmp280 operating mode.
 * @param[in] bus_handle Initialized I2C bus handle.
 *
 * @return
 *      - ESP_OK: BMP280 initialized successfully.
 *      - ESP_ERR_INVALID_ARG: bmp280 or bus_handle is NULL.
 *      - ESP_ERR_TIMEOUT: I2C communication timed out.
 *      - ESP_FAIL: Failed to read calibration data or configure sensor.
 */
esp_err_t bmp280_initialize(bmp280_t* bmp280, bmp280_mode_t work_mode,
                            i2c_master_bus_handle_t bus_handle);

/**
 * @brief read from bmp280 data registers.
 *
 * @note bmp280_initialize() must be called successfully before this function
 *
 * @param[in] bmp280 bmp280 handle.
 * @param[out] measurements bmp280 measurements.
 *
 * @return
 *      - ESP_OK: data read successfully.
 */
esp_err_t bmp280_read(bmp280_t* bmp280, bmp280_measurements* measurements);

/**
 * @brief make bmp280 take measurements in FORCE mode
 *
 * @param[in] bmp280 bmp280 handle.
 * @param[in] timeout_ms measurement timeout in ms.
 *
 * @return
 *      - ESP_OK: measurement has been taken.
 */
esp_err_t bmp280_force_measurement(bmp280_t* bmp280, const uint16_t timeout_ms);

/**
 * @brief resets bmp280 device.
 *
 * device is reset using the compete power-on-reset procedure.
 *
 * @param[in] bmp280 bmp280 handle.
 *
 * @return
 *      - ESP_OK: device reset successfully.
 *      - ESP_ERR_INVALID_ARG: bmp280 is NULL.
 */
esp_err_t bmp280_reset(bmp280_t* bmp280);

/** @} */
#endif