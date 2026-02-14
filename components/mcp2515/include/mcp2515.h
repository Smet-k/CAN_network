#ifndef MCP2515_H
#define MCP2515_H
/**
 * @defgroup mcp2515 MCP2515 Driver
 * @brief Driver for MCP2515 CAN bus
 * @{
 */
#include "driver/gpio.h"
#include "driver/spi_master.h"

#define MCP2515_ROLLOVER_ENABLE 0x01
#define MCP2515_ROLLOVER_DISABLE 0x00
#define MCP2515_EXIDE_OFF 0x00
#define MCP2515_EXIDE_ON 0x01

/**
 * @brief MCP2515 RXM options.
 * options to turn on/off filters and masks.
 */
typedef enum {
    MCP2515_RXM_ON = 0x00, /** masks/filters ON */
    MCP2515_RXM_OFF = 0x03 /** masks/filters OFF */
} mcp2515_rxm_t;

/**
 * @brief MCP2515 configuration.
 */
typedef struct {
    spi_host_device_t spi_host;
    int clock_hz;
    gpio_num_t gpio_cs;
    gpio_num_t gpio_int;
    struct {
        uint8_t rollover;
        mcp2515_rxm_t rx_mode;
        uint8_t RX0_EXIDE;
        uint8_t RX1_EXIDE;
    } flags;
} mcp2515_config_t;

/**
 * @brief MCP2515 handle.
 */
typedef struct {
    spi_device_handle_t spi;
    gpio_num_t gpio_int;
} mcp2515_handle_t;

/**
 * @brief Frequencies that can be used by MCP2515.
 */
typedef enum {
    MCP2515_CRYSTAL_8MHZ,
    MCP2515_CRYSTAL_16MHZ
} mcp2515_crystal_t;

/**
 * @brief Set of commonly used bitrates.
 */
typedef enum {
    MCP2515_BITRATE_125KBPS,
    MCP2515_BITRATE_250KBPS,
    MCP2515_BITRATE_500KBPS,
    MCP2515_BITRATE_1MBPS
} mcp2515_bitrate_t;

/**
 * @brief Configuration of timings for MCP2515.
 */
typedef struct{
    mcp2515_crystal_t crystal;
    mcp2515_bitrate_t bitrate;
    uint8_t cnf1;
    uint8_t cnf2;
    uint8_t cnf3;
} mcp2515_timing_cfg_t;

/**
 * @brief CAN bus message format
 */
typedef enum {
    MCP_FRAME_STD = 0,
    MCP_FRAME_EXT = 1
} mcp2515_frame_format_t;

/**
 * @brief Possible MCP2515 operation modes.
 */
typedef enum {
    MCP2515_OPMODE_NORMAL = 0x00,
    MCP2515_OPMODE_SLEEP = 0x20,
    MCP2515_OPMODE_LOOPBACK = 0x40,
    MCP2515_OPMODE_LISTEN_ONLY = 0x60,
    MCP2515_OPMODE_CONFIGURATION = 0x80
} mcp2515_opmode_t;

/**
 * @brief Priorities of MCP2515 messages
 */
typedef enum {
    MCP2515_PRIORITY_LOWEST =0x00,
    MCP2515_PRIORITY_LOW,
    MCP2515_PRIORITY_HIGH,
    MCP2515_PRIORITY_HIGHEST
} mcp2515_priority_t;

/**
 * @brief MCP2515 masks.
 */
typedef enum {
    MCP2515_MASK0 = 0,
    MCP2515_MASK1 = 1
} mcp_mask_t;


/**
 * @brief MCP2515 filters.
 */
typedef enum {
    MCP2515_FILTER0 = 0x00,
    MCP2515_FILTER1,
    MCP2515_FILTER2,
    MCP2515_FILTER3,
    MCP2515_FILTER4,
    MCP2515_FILTER5
} mcp_filter_t;

/**
 * @brief MCP2515 transmit buffers.
 * 
 * used to pick which buffer will be used by certain functions.
 */
typedef enum {
    MCP2515_TXB0 = 0x00,
    MCP2515_TXB1,
    MCP2515_TXB2
} mcp_txb_t;

/**
 * @brief MCP2515 receive buffers.
 * 
 * used to pick which buffer will be used by certain functions.
 */
typedef enum {
    MCP2515_RXB0 = 0x00,
    MCP2515_RXB1
} mcp_rxb_t;

// maybe make it universal for both transmit and receive
/**
 * @brief MCP2515 received message frame.
 */
typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
} mcp2515_frame_t;

/**
 * @brief Initialize MCP2515 device.
 * 
 * The MCP2515 handle must remain valid for the lifetime of the device usage.
 * @param[out] mcp pointer to MCP2515 handle.
 * @param[in] cfg MCP2515 configuration.
 * 
 * @return
 *      - ESP_OK: MCP2515 initialized successfully.
 *      - ESP_ERR_INVALID_ARG: mcp, cfg is NULL or cfg's cloch_hz is <= 0.
 */
esp_err_t mcp2515_init(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg);

/**
 * @brief Reset MCP2515.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * 
 * @return
 *      - ESP_OK: MCP2515 reset successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL.
 */
esp_err_t mcp2515_reset(mcp2515_handle_t* mcp);

/**
 * @brief Wakes up the MCP2515.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * 
 * @return
 *      - ESP_OK: MCP2515 wake up is successful.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL.
 */
esp_err_t mcp2515_wake_up(mcp2515_handle_t* mcp);

/**
 * @brief Set MCP2515 operation mode.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] mode new MCP2515 operation mode.
 * 
 * @return
 *      - ESP_OK: operation mode set successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL or provided operation mode doesn't fit the mask.
 *      - ESP_ERR_TIMEOUT: operation mode change timed out.
 * 
 * @note To change to the mode other than CONFIGURE, device must be in CONFIGURE operation mode.
 */
esp_err_t mcp2515_set_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t mode);

/**
 * @brief Get MCP2515 current operation mode.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[out] mode current MCP2515 operation mode.
 * 
 * @return
 *      - ESP_OK: operation mode read successfully.
 *      - ESP_ERR_INVALID_MODE: mcp or mode is NULL.
 */
esp_err_t mcp2515_get_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t* mode);

/**
 * @brief Transmit a CAN bus message.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] frame message frame to be transmit.
 * 
 * @return
 *      - ESP_OK: message transmission was successful.
 *      - ESP_ERR_INVALID_ARG: mcp or dlc is NULL.
 *      - ESP_FAIL: message transmission failed.
 */
esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, const mcp2515_frame_t* frame);

/**
 * @brief Receive a CAN bus message.
 * 
 * @param[in] mcp pointer in MCP2515 handle.
 * @param[out] frame received canbus message.
 * 
 * @return
 *      - ESP_OK: message received successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or frame is NULL.
 *      - ESP_ERR_NOT_FOUND: no messages to receive or receival failed.
 * 
 * @note MCP2515 CAN bus timings must be set up (mcp2515_configure_timing()) before message receival.
 */
esp_err_t mcp2515_receive(mcp2515_handle_t* mcp, mcp2515_frame_t* frame);

/**
 * @brief Configure MCP2515 CAN bus timings.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] crystal MCP2515 frequency.
 * @param[in] bitrate CAN bus bitrate.
 * 
 * @return
 *      - ESP_OK: CAN bus timings configured successfully.
 *      - ESP_ERR_INVALID_ARG: mcp is NULL.
 *      - ESP_FAIL: CAN bus timing configuration failed.
 */
esp_err_t mcp2515_configure_timing(mcp2515_handle_t* mcp, mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate);
/** @} */

/**
 * @brief set CAN bus filters.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] filter_reg filter register to be set.
 * @param[in] filter filter value.
 * 
 * @return
 *      - ESP_OK: filters set successfully.
 *      - ESP_ERR_INVALID_STATE: device is not in configuration mode.
 */
esp_err_t mcp2515_set_filter(mcp2515_handle_t* mcp, mcp_filter_t filter_reg, uint16_t filter);

/**
 * @brief Sets a mask for provided register.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] mask_reg mask register to be set.
 * @param[in] mask mask value.
 * 
 * @return
 *      - ESP_OK: mask set successfully.
 *      - ESP_ERR_INVALID_ARG: mcp is NULL.
 *      - ESP_ERR_INVALID_STATE: device is not in configuration mode.
 */
esp_err_t mcp2515_set_mask(mcp2515_handle_t* mcp, mcp_mask_t mask_reg, uint16_t mask);

#endif