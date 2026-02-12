#include "mcp2515.h"

#define MCP_CANSTAT 0x0E
#define MCP_CANCTRL 0x0F
#define MCP_OPMOD_MASK 0xE0
#define MCP_OPMOD_SLEEP 0x20
#define MCP_FILTER_OFF_MASK 0x60

#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2A

#define MCP_BRP_CNF_MASK 0x3F
#define MCP_SJW_CNF_MASK 0xC0

#define MCP_CANINTE 0x2B
#define MCP_RX0IE 0x01
#define MCP_RX1IE 0x02
#define MCP_CANINTF 0x2C
#define MCP_CANINTE_WAKIE (1 << 6)
#define MCP_RX0IF 0x01
#define MCP_RX1IF 0x02
#define MCP_EFLG 0x2D
#define MCP_TEC 0x1C

// RX buffers
#define MCP_RBXnCTRL_BUKT(bit) ((bit) << 2)
#define MCP_RBXnCTRL_RXM(bit) ((bit) << 5)
#define MCP_RBXnCTRL(base) ((base) + 0x00)
#define MCP_RXBnSIDH(base) ((base) + 0x01)
#define MCP_RXBnSIDL(base) ((base) + 0x02)
#define MCP_RXBnEID8(base) ((base) + 0x03)
#define MCP_RXBnEID0(base) ((base) + 0x04)
#define MCP_RXBnDLC(base)  ((base) + 0x05)
#define MCP_RXBnDATA(base) ((base) + 0x06)

// TX BUFFERS
#define MCP_TXB_SIDH(base) ((base) + 0x01)
#define MCP_TXB_SIDL(base) ((base) + 0x02)
#define MCP_TXB_EID8(base) ((base) + 0x03)
#define MCP_TXB_EID0(base) ((base) + 0x04)
#define MCP_TXB_DLC(base)  ((base) + 0x05)
#define MCP_TXB_D0(base)   ((base) + 0x06)

#define MCP_TXB_TXREQ (1 << 3)
#define MCP_TXB_TXERR (1 << 4)
#define MCP_TXB_MLOA (1 << 5)

#define MCP_RBXnCTRL_BUKT_MASK  (1 << 2)
#define MCP_RBXnCTRL_RXM_MASK   (3 << 5)
#define MCP_EXIDE_MASK (1 << 3)
#define MCP_EXIDE(bit) ((bit) << 3)

#define MCP_RXFnSIDL(base) ((base) +0x01)

#define MCP_SIDH_ID_MASK 0xFF
#define MCP_SIDL_ID_MASK 0x07

#define MCP_CMD_RESET 0xC0
#define MCP_CMD_READ 0x03
#define MCP_CMD_WRITE 0x02
#define MCP_CMD_READ_STATUS 0xA0
#define MCP_CMD_RX_STATUS 0xB0
#define MCP_CMD_BIT_MODIFY 0x05

static const mcp2515_timing_cfg_t timing_table[] = {
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_500KBPS, 0x00, 0x90, 0x02},
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_250KBPS, 0x00, 0xA1, 0x06},
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_125KBPS, 0x01, 0xA1, 0x06},

    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_500KBPS, 0x00, 0xD1, 0x81},
    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_250KBPS, 0x01, 0xD1, 0x81},
    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_125KBPS, 0x03, 0xD1, 0x81},
};

static const uint8_t mcp_txb_base[3] = {
    0x30,
    0x40,
    0x50
};

static const uint8_t mcp_rxb_base[2] = {
    0x60,
    0x70
};

static const uint8_t mcp_rxm_base_addr[2] = {
    0x20,
    0x24
};

static const uint8_t rxf_sidh_addr[6] = {
    0x00, 0x04, 0x08,
    0x10, 0x14, 0x18
};

/**
 * @brief Read value from MCP2515 register.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] reg register to read from.
 * @param[out] val pointer to variable to write into.
 * 
 * @return
 *      - ESP_OK: register read successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL.
 */
static esp_err_t mcp2515_read_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t* val);

/**
 * @brief Write value into MCP2515 register.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] reg register to write into.
 * @param[in] val value to write.
 * 
 * @return
 *      - ESP_OK: value written successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL.
 */
static esp_err_t mcp2515_write_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t val);

/**
 * @brief Modify value of the register.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] reg register to modify.
 * @param[in] mask mask for data to use.
 * @param[in] data data to be written with mask.
 * 
 * @return
 *      - ESP_OK: register modifier successfully.
 *      - ESP_ERR_INVALID_ARG: mcp or mcp->spi is NULL.
 */
static esp_err_t mcp2515_bit_modify(mcp2515_handle_t* mcp, uint8_t reg,
                                    uint8_t mask, uint8_t data);

/**
 * @brief Check if MCP2515 is awake.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * 
 * @return
 *      - TRUE device is sleeping.
 *      - FALSE device is awake.
 */
static bool mcp2515_is_awake(mcp2515_handle_t* mcp);

/**
 * @brief Find a viable timings configuration.
 * 
 * Timing configuration is found by matching frequency and bitrate
 * with timing table.
 * 
 * @param[in] crystal MCP2515 crystal frequency.
 * @param[out] bitrate Chosen message bitrate.
 * 
 * @return viable timings from table or NULL if no timings is found.
 */
static const mcp2515_timing_cfg_t* mcp2515_find_timing(mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate);

/**
 * @brief Set multiple register configurations.
 * 
 * Sets multiple settings in registers, following mcp2515_config_t flags.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * @param[in] cfg MCP2515 configuration.
 * 
 * @return
 *      - ESP_OK: configuration was successful.
 *      - ESP_ERR_INVALID_ARG: mcp or cfg is NULL.
 */
static esp_err_t mcp2515_ctrl_reg_cfg(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg);

esp_err_t mcp2515_init(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg) {
    if (!mcp || !cfg || cfg->clock_hz <= 0)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = cfg->clock_hz,
        .mode = 0,
        .spics_io_num = cfg->gpio_cs,
        .queue_size = 7,
    };

    gpio_config_t cs_conf = {
        .pin_bit_mask = 1ULL << cfg->gpio_cs,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&cs_conf);
    gpio_set_level(cfg->gpio_cs, 1);

    err = spi_bus_add_device(cfg->spi_host, &devcfg, &mcp->spi);
    if (err != ESP_OK)
        return err;

    mcp->gpio_int = cfg->gpio_int;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << cfg->gpio_int,
        .mode = GPIO_MODE_INPUT,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return err;

    err = mcp2515_reset(mcp);
    if(err != ESP_OK)
        return err;

    return mcp2515_ctrl_reg_cfg(mcp, cfg);
}

static esp_err_t mcp2515_ctrl_reg_cfg(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg){
    if(!mcp || !cfg) return ESP_ERR_INVALID_ARG;
    esp_err_t err;

    err = mcp2515_bit_modify(mcp, MCP_RBXnCTRL(mcp_rxb_base[0]),
        MCP_RBXnCTRL_BUKT_MASK, MCP_RBXnCTRL_BUKT(cfg->flags.rollover));
    if(err != ESP_OK) return err;
    err = mcp2515_bit_modify(mcp, MCP_RBXnCTRL(mcp_rxb_base[0]),
        MCP_RBXnCTRL_RXM_MASK, MCP_RBXnCTRL_RXM(cfg->flags.rx_mode));
    if(err != ESP_OK) return err;

    err = mcp2515_bit_modify(mcp, MCP_RXBnSIDL(mcp_rxb_base[0]),
        MCP_EXIDE_MASK, MCP_EXIDE(cfg->flags.RX0_EXIDE));
    if(err != ESP_OK) return err;
    err = mcp2515_bit_modify(mcp, MCP_RXBnSIDL(mcp_rxb_base[0]),
        MCP_EXIDE_MASK, MCP_EXIDE(cfg->flags.RX1_EXIDE));
    if(err != ESP_OK) return err;


    err = mcp2515_bit_modify(mcp, MCP_TXB_SIDL(mcp_rxb_base[0]),
        MCP_EXIDE_MASK, MCP_EXIDE(cfg->flags.TX_EXIDE));
    if(err != ESP_OK) return err;

    return ESP_OK;
}

esp_err_t mcp2515_reset(mcp2515_handle_t* mcp) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;

    uint8_t cmd = MCP_CMD_RESET;

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };

    esp_err_t err = spi_device_transmit(mcp->spi, &t);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

static esp_err_t mcp2515_read_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t* val) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;

    uint8_t tx_buf[3] = {MCP_CMD_READ, reg, 0x00};
    uint8_t rx_buf[3] = {0};
    spi_transaction_t t = {
        .length = 8 * sizeof(tx_buf), .tx_buffer = tx_buf, .rx_buffer = rx_buf};

    esp_err_t err = spi_device_transmit(mcp->spi, &t);
    if (err != ESP_OK)
        return err;

    *val = rx_buf[2];
    return ESP_OK;
}

static esp_err_t mcp2515_write_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t val) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;

    uint8_t tx_buf[3] = {MCP_CMD_WRITE, reg, val};
    spi_transaction_t t = {.length = 8 * sizeof(tx_buf), .tx_buffer = tx_buf};

    return spi_device_transmit(mcp->spi, &t);
}

static esp_err_t mcp2515_bit_modify(mcp2515_handle_t* mcp, uint8_t reg, uint8_t mask,
                             uint8_t data) {
    if(!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;
    
    uint8_t tx[4] = {MCP_CMD_BIT_MODIFY, reg, mask, data};
    spi_transaction_t t = {
        .length = 8 * sizeof(tx),
        .tx_buffer = tx,
    };
    return spi_device_transmit(mcp->spi, &t);
}

static bool mcp2515_is_awake(mcp2515_handle_t* mcp) {
    uint8_t canstat;
    if (mcp2515_read_reg(mcp, MCP_CANSTAT, &canstat) != ESP_OK)
        return false;

    return ((canstat & MCP_OPMOD_MASK) != MCP_OPMOD_SLEEP);
}

esp_err_t mcp2515_wake_up(mcp2515_handle_t* mcp) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    err = mcp2515_bit_modify(mcp, MCP_CANINTE, MCP_CANINTE_WAKIE,
                             MCP_CANINTE_WAKIE);
    if (err != ESP_OK)
        return err;

    mcp2515_set_opmode(mcp, MCP_OPMODE_CONFIGURATION);

    vTaskDelay(pdMS_TO_TICKS(1));

    if (!mcp2515_is_awake(mcp))
        return ESP_ERR_TIMEOUT;

    return ESP_OK;
}

esp_err_t mcp2515_set_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t mode) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;

    if (mode & ~MCP_OPMOD_MASK)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    err = mcp2515_bit_modify(mcp, MCP_CANCTRL, MCP_OPMOD_MASK, mode);
    if (err != ESP_OK)
        return err;

    vTaskDelay(pdMS_TO_TICKS(1));

    uint8_t canstat;
    err = mcp2515_read_reg(mcp, MCP_CANSTAT, &canstat);
    if (err != ESP_OK)
        return err;

    for (int i = 0; i < 100; i++) {
        uint8_t canstat;
        err = mcp2515_read_reg(mcp, MCP_CANSTAT, &canstat);
        if (err != ESP_OK)
            return err;

        if ((canstat & MCP_OPMOD_MASK) == mode)
            return ESP_OK;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t mcp2515_get_opmode(mcp2515_handle_t* mcp, mcp2515_opmode_t* mode) {
    if (!mcp || !mode)
        return ESP_ERR_INVALID_ARG;

    uint8_t canstat;
    esp_err_t err = mcp2515_read_reg(mcp, MCP_CANSTAT, &canstat);
    if (err != ESP_OK)
        return err;

    *mode = (mcp2515_opmode_t)(canstat & MCP_OPMOD_MASK);
    return ESP_OK;
}

esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, uint16_t id, uint8_t dlc, const uint8_t* data) {
    if (!mcp || dlc > 8 ) return ESP_ERR_INVALID_ARG;

    uint8_t sidh = (id >> 3) & MCP_SIDH_ID_MASK;
    uint8_t sidl = (id & MCP_SIDL_ID_MASK) << 5;

    mcp2515_write_reg(mcp, MCP_TXB_SIDH(mcp_txb_base[0]), sidh);
    mcp2515_write_reg(mcp, MCP_TXB_SIDL(mcp_txb_base[0]), sidl);
    mcp2515_write_reg(mcp, MCP_TXB_EID8(mcp_txb_base[0]), 0);
    mcp2515_write_reg(mcp, MCP_TXB_EID0(mcp_txb_base[0]), 0);
    mcp2515_write_reg(mcp, MCP_TXB_DLC(mcp_txb_base[0]), dlc);

    for (int i = 0; i < dlc; i++) {
        mcp2515_write_reg(mcp, MCP_TXB_D0(mcp_txb_base[0]) + i, data[i]);
    }

    mcp2515_bit_modify(mcp, mcp_txb_base[0], MCP_TXB_TXREQ, MCP_TXB_TXREQ);

    uint8_t ctrl;

    do {
        mcp2515_read_reg(mcp, mcp_txb_base[0], &ctrl);
    } while (ctrl & MCP_TXB_TXREQ);

    if (ctrl & MCP_TXB_TXERR)
        return ESP_FAIL;

    return ESP_OK;
}

esp_err_t mcp2515_set_filters(mcp2515_handle_t* mcp) {
    if (!mcp) return ESP_ERR_INVALID_ARG;
    // for now turn off the filters, proper filters will be added later
    mcp2515_write_reg(mcp, MCP_RBXnCTRL(mcp_rxb_base[0]), 0x60);
    mcp2515_write_reg(mcp, MCP_RBXnCTRL(mcp_rxb_base[1]), 0x60);

    mcp2515_write_reg(mcp, MCP_RXBnSIDH(mcp_rxb_base[0]), 0x00);
    mcp2515_write_reg(mcp, MCP_RXBnSIDL(mcp_rxb_base[0]), 0x00);
    mcp2515_write_reg(mcp, MCP_RXBnSIDH(mcp_rxb_base[1]), 0x00);
    mcp2515_write_reg(mcp, MCP_RXBnSIDL(mcp_rxb_base[1]), 0x00);

    mcp2515_bit_modify(mcp, MCP_CANINTF,
                       MCP_RX0IF | MCP_RX1IF,
                       0);

    mcp2515_write_reg(mcp, MCP_CANINTE,
                      MCP_RX0IE | MCP_RX1IE);

    return ESP_OK;
}

esp_err_t mcp2515_configure_timing(mcp2515_handle_t* mcp, mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate){
    if (!mcp) return ESP_ERR_INVALID_ARG;
    const mcp2515_timing_cfg_t* tcfg = mcp2515_find_timing(crystal, bitrate); 
    mcp2515_write_reg(mcp, MCP_CNF1, tcfg->cnf1);
    mcp2515_write_reg(mcp, MCP_CNF2, tcfg->cnf2);
    mcp2515_write_reg(mcp, MCP_CNF3, tcfg->cnf3);

    uint8_t cnf1, cnf2, cnf3;
    mcp2515_read_reg(mcp, MCP_CNF1, &cnf1);
    if (cnf1 != tcfg->cnf1) return ESP_FAIL;
    mcp2515_read_reg(mcp, MCP_CNF2, &cnf2);
    if (cnf2 != tcfg->cnf2) return ESP_FAIL;
    mcp2515_read_reg(mcp, MCP_CNF3, &cnf3);
    if (cnf3 != tcfg->cnf3) return ESP_FAIL;

    return ESP_OK;
}

esp_err_t mcp2515_receive(mcp2515_handle_t* mcp, mcp2515_frame_t* frame) {
    if (!mcp || !frame) return ESP_ERR_INVALID_ARG;

    uint8_t intf;
    mcp2515_read_reg(mcp, MCP_CANINTF, &intf);

    if (!(intf & MCP_RX0IF))
        return ESP_FAIL;
 
    uint8_t sidh, sidl, dlc;
    mcp2515_read_reg(mcp, MCP_RXBnSIDH(mcp_rxb_base[0]), &sidh);
    mcp2515_read_reg(mcp, MCP_RXBnSIDL(mcp_rxb_base[0]), &sidl);
    mcp2515_read_reg(mcp, MCP_RXBnDLC(mcp_rxb_base[0]), &dlc);

    frame->id = (sidh << 3) + (sidl >> 5);
    frame->dlc = dlc;

    for (int i = 0; i < (dlc & 0x0F); i++) {
        mcp2515_read_reg(mcp, MCP_RXBnDATA(mcp_rxb_base[0]) + i, &frame->data[i]);
    }
    mcp2515_bit_modify(mcp, MCP_CANINTF, MCP_RX0IF, 0);

    return ESP_OK;
}

static const mcp2515_timing_cfg_t* mcp2515_find_timing(mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate) {
    size_t size = (sizeof(timing_table) / sizeof(timing_table[0]));
    for (size_t i = 0; i < size; i++) {
        if (timing_table[i].crystal == crystal &&
            timing_table[i].bitrate == bitrate) {
            return &timing_table[i]; // note down that pointer is used to not copy the values but instead point to them 
        }
    }
    return NULL;
}

// RXM0 RXF0 RXF1
// RXM1 RXF2 RXF3 RXF4 RXF5