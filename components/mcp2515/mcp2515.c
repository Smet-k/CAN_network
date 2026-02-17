#include "mcp2515.h"

#define MCP_CANSTAT 0x0E
#define MCP_CANCTRL 0x0F
#define MCP_OPMOD_MASK 0xE0
#define MCP_OPMOD_SLEEP 0x20

#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2A

#define MCP_CANINTE 0x2B
#define MCP_CANINTF 0x2C
#define MCP_CANINTE_WAKIE (1 << 6)

// RX buffers
#define MCP_RXBnCTRL_BUKT(bit) ((bit) << 2)
#define MCP_RXBnCTRL_RXM(bit) ((bit) << 5)
#define MCP_RXBnCTRL(base) ((base) + 0x00)
#define MCP_RXBnSIDH(base) ((base) + 0x01)
#define MCP_RXBnSIDL(base) ((base) + 0x02)
#define MCP_RXBnEID8(base) ((base) + 0x03)
#define MCP_RXBnEID0(base) ((base) + 0x04)
#define MCP_RXBnDLC(base)  ((base) + 0x05)
#define MCP_RXBnDATA(base) ((base) + 0x06)

#define MCP_RXMnSIDH(base) ((base) + 0x00)
#define MCP_RXMnSIDL(base) ((base) + 0x01)
#define MCP_RXMnEID8(base) ((base) + 0x02)
#define MCP_RXMnEID0(base) ((base) + 0x03)

#define MCP_RXFnSIDH(base) ((base) + 0x00)
#define MCP_RXFnSIDL(base) ((base) + 0x01)
#define MCP_RXFnEID8(base) ((base) + 0x02)
#define MCP_RXFnEID0(base) ((base) + 0x03)

#define MCP_RXBnCTRL_BUKT_MASK  (1 << 2)
#define MCP_RXBnCTRL_RXM_MASK   (3 << 5)
#define MCP_RXnSIDL(base) ((base) +0x01)

#define MCP_RX0IF 0x01
#define MCP_RX1IF 0x02
#define MCP_RX0IE 0x01
#define MCP_RX1IE 0x02

// TX BUFFERS
#define MCP_TXBnSIDH(base) ((base) + 0x01)
#define MCP_TXBnSIDL(base) ((base) + 0x02)
#define MCP_TXBnEID8(base) ((base) + 0x03)
#define MCP_TXBnEID0(base) ((base) + 0x04)
#define MCP_TXBnDLC(base)  ((base) + 0x05)
#define MCP_TXBnD0(base)   ((base) + 0x06)

#define MCP_TXBnTXREQ (1 << 3)
#define MCP_TXBnTXERR (1 << 4)
#define MCP_TXBnMLOA (1 << 5)

// UNIVERSAL
#define MCP_EXIDE_MASK (1 << 3)
#define MCP_EXIDE(bit) ((bit) << 3)

#define MCP_SIDH_ID_MASK 0xFF
#define MCP_EIDn_ID_MASK 0xFF
#define MCP_SIDL_ID_MASK 0x07
#define MCP_SIDL_EXT_ID_MASK1 0x03
#define MCP_SIDL_EXT_ID_MASK2 0xE0
#define MCP_EID8_OFFSET    0x08

// COMMANDS
#define MCP_CMD_RESET 0xC0
#define MCP_CMD_READ 0x03
#define MCP_CMD_WRITE 0x02
#define MCP_CMD_BIT_MODIFY 0x05

static const mcp2515_timing_cfg_t timing_table[] = {
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_500KBPS, 0x00, 0x90, 0x02},
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_250KBPS, 0x01, 0x90, 0x02}, 
    {MCP2515_CRYSTAL_8MHZ, MCP2515_BITRATE_125KBPS, 0x01, 0xB1, 0x05},
    
    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_500KBPS, 0x00, 0xB1, 0x05},
    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_250KBPS, 0x01, 0xB1, 0x05},
    {MCP2515_CRYSTAL_16MHZ, MCP2515_BITRATE_125KBPS, 0x03, 0xB1, 0x05},
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

static const uint8_t mcp_rxf_sidh_addr[6] = {
    0x00, 0x04, 0x08,
    0x10, 0x14, 0x18
};

typedef struct {
    uint8_t sidh;
    uint8_t sidl;
    uint8_t eid8;
    uint8_t eid0;
} mcp2515_id_t;

/**
 * @brief packs provided id into transmission compatible format.
 * 
 * @param[in] id packed id.
 * @param[in] extended is id extended.
 * 
 * @return packed id struct.
 */
static mcp2515_id_t mcp2515_pack_id(uint32_t id, bool extended);

/**
 * @brief unpacks provided id into integer.
 * 
 * @param[in] id id to be unpacked.
 * @param[in] extended is id extended.
 * 
 * @return id integer.
 */
static uint32_t mcp2515_unpack_id(mcp2515_id_t id, bool extended);

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
 * @brief returns index of a free TX buffer.
 * 
 * @param[in] mcp pointer to mcp handle.
 * 
 * @return
 *      - positive number if any buffer is free.
 *      - negative if there is no free buffers (-1).
 */
static int8_t mcp2515_get_free_tx_buffer(mcp2515_handle_t* mcp);

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

/**
 * @brief Set interrupt configurations.
 * 
 * @param[in] mcp pointer to MCP2515 handle.
 * 
 * @return
 *      - ESP_OK: configuration was successful.
 */
static esp_err_t mcp2515_configure_interrupts(mcp2515_handle_t* mcp);

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

    err = mcp2515_ctrl_reg_cfg(mcp, cfg);
    if(err != ESP_OK)
        return err;
    
    return mcp2515_configure_interrupts(mcp);
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

esp_err_t mcp2515_wake_up(mcp2515_handle_t* mcp) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    err = mcp2515_bit_modify(mcp, MCP_CANINTE, MCP_CANINTE_WAKIE,
                             MCP_CANINTE_WAKIE);
    if (err != ESP_OK)
        return err;

    mcp2515_set_opmode(mcp, MCP2515_OPMODE_CONFIGURATION);

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

esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, const mcp2515_frame_t* frame) {
    if (!mcp || frame->dlc > 8 ) return ESP_ERR_INVALID_ARG;
    esp_err_t err;

    int8_t buf = mcp2515_get_free_tx_buffer(mcp);
    if(buf < 0) return ESP_FAIL;

    uint8_t base = mcp_txb_base[buf];
    mcp2515_id_t id = mcp2515_pack_id(frame->id, frame->extended);

    err = mcp2515_write_reg(mcp, MCP_TXBnSIDH(base), id.sidh);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_TXBnSIDL(base), id.sidl);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_TXBnEID8(base), id.eid8);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_TXBnEID0(base), id.eid0);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_TXBnDLC(base), frame->dlc);
    if(err != ESP_OK) return err;

    for (int i = 0; i < frame->dlc; i++) {
        err = mcp2515_write_reg(mcp, MCP_TXBnD0(base) + i, frame->data[i]);
        if(err != ESP_OK) return err;
    }

    uint8_t ctrl;

    err = mcp2515_read_reg(mcp, base, &ctrl);
    if(err != ESP_OK) return err;

    if (ctrl & MCP_TXBnTXERR)
        return ESP_FAIL;

    return mcp2515_bit_modify(mcp, base, MCP_TXBnTXREQ, MCP_TXBnTXREQ);
}

esp_err_t mcp2515_receive(mcp2515_handle_t* mcp, mcp2515_frame_t* frame) {
    if (!mcp || !frame) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    uint8_t intf;
    err = mcp2515_read_reg(mcp, MCP_CANINTF, &intf);
    if(err != ESP_OK) return err;

    int buf = -1;

    if (intf & MCP_RX0IF) buf = 0;
    else if (intf & MCP_RX1IF) buf = 1;
    else return ESP_ERR_NOT_FOUND;
 
    uint8_t base = mcp_rxb_base[buf];

    mcp2515_id_t id = {0};
    uint8_t dlc;
    err = mcp2515_read_reg(mcp, MCP_RXBnSIDH(base), &id.sidh);
    if(err != ESP_OK) return err;
    err = mcp2515_read_reg(mcp, MCP_RXBnSIDL(base), &id.sidl);
    if(err != ESP_OK) return err;
    err = mcp2515_read_reg(mcp, MCP_RXBnEID8(base), &id.eid8);
    if(err != ESP_OK) return err;
    err = mcp2515_read_reg(mcp, MCP_RXBnEID0(base), &id.eid0);
    if(err != ESP_OK) return err;
    err = mcp2515_read_reg(mcp, MCP_RXBnDLC(base), &dlc);
    if(err != ESP_OK) return err;

    frame->extended = (id.sidl & MCP_EXIDE_MASK) != 0;
    frame->dlc = dlc & 0x0F;

    frame->id = mcp2515_unpack_id(id, frame->extended);

    for (int i = 0; i < (dlc & 0x0F); i++) {
        err = mcp2515_read_reg(mcp, MCP_RXBnDATA(base) + i, &frame->data[i]);
        if(err != ESP_OK) return err;
    }

    return mcp2515_bit_modify(mcp, MCP_CANINTF,
        buf == 0 ? MCP_RX0IF : MCP_RX1IF,
        0);
}

esp_err_t mcp2515_set_filter(mcp2515_handle_t* mcp, mcp_filter_t filter_reg, uint32_t filter, bool extended){
    if (!mcp) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    mcp2515_opmode_t opmode;

    err = mcp2515_get_opmode(mcp, &opmode);
    if (err != ESP_OK)
            return err;
    
    if (opmode != MCP2515_OPMODE_CONFIGURATION){
        err = mcp2515_set_opmode(mcp, MCP2515_OPMODE_CONFIGURATION);
        if(err != ESP_OK) return err;
    }

    mcp2515_id_t id = mcp2515_pack_id(filter, extended);

    uint8_t base = mcp_rxf_sidh_addr[filter_reg];

    err = mcp2515_write_reg(mcp, MCP_RXFnSIDH(base), id.sidh);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXFnSIDL(base), id.sidl);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXFnEID8(base), id.eid8);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXFnEID0(base), id.eid0);
    if(err != ESP_OK) return err;

    if(opmode != MCP2515_OPMODE_CONFIGURATION)
        return mcp2515_set_opmode(mcp, opmode);

    return ESP_OK;
}

esp_err_t mcp2515_set_mask(mcp2515_handle_t* mcp, mcp_mask_t mask_reg, uint32_t mask, bool extended){
    if(!mcp) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    mcp2515_opmode_t opmode;

    err = mcp2515_get_opmode(mcp, &opmode);
    if (err != ESP_OK)
            return err;
    
    if (opmode != MCP2515_OPMODE_CONFIGURATION)
        mcp2515_set_opmode(mcp, MCP2515_OPMODE_CONFIGURATION);
    
    mcp2515_id_t id = mcp2515_pack_id(mask, extended);

    uint8_t base = mcp_rxm_base_addr[mask_reg];

    err = mcp2515_write_reg(mcp, MCP_RXMnSIDH(base), id.sidh);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXMnSIDL(base), id.sidl);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXMnEID8(base), id.eid8);
    if(err != ESP_OK) return err;
    err = mcp2515_write_reg(mcp, MCP_RXMnEID0(base), id.eid0);
    if(err != ESP_OK) return err;

    if(opmode != MCP2515_OPMODE_CONFIGURATION)
        return mcp2515_set_opmode(mcp, opmode);
    
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

esp_err_t mcp2515_create_frame(mcp2515_frame_t* frame, uint8_t* data, uint8_t dlc, uint32_t id, bool extended){
    if(!frame || !data || dlc > 8) return ESP_ERR_INVALID_ARG;

    frame->id = id;
    frame->dlc = dlc;
    frame->extended = extended;

    for (int i = 0; i < dlc; i++)
        frame->data[i] = data[i];

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

static const mcp2515_timing_cfg_t* mcp2515_find_timing(mcp2515_crystal_t crystal, mcp2515_bitrate_t bitrate) {
    size_t size = (sizeof(timing_table) / sizeof(timing_table[0]));
    for (size_t i = 0; i < size; i++) {
        if (timing_table[i].crystal == crystal &&
            timing_table[i].bitrate == bitrate) {
            return &timing_table[i];
        }
    }
    return NULL;
}

static esp_err_t mcp2515_configure_interrupts(mcp2515_handle_t* mcp){
    esp_err_t err;

    err = mcp2515_bit_modify(mcp, MCP_CANINTF,
                       MCP_RX0IF | MCP_RX1IF,
                       0);
    if (err != ESP_OK) return err;

    return mcp2515_write_reg(mcp, MCP_CANINTE,
                      MCP_RX0IE | MCP_RX1IE);
}

static esp_err_t mcp2515_ctrl_reg_cfg(mcp2515_handle_t* mcp, const mcp2515_config_t* cfg){
    if(!mcp || !cfg) return ESP_ERR_INVALID_ARG;
    esp_err_t err;

    err = mcp2515_bit_modify(mcp, MCP_RXBnCTRL(mcp_rxb_base[0]),
        MCP_RXBnCTRL_BUKT_MASK, MCP_RXBnCTRL_BUKT(cfg->flags.rollover));
    if(err != ESP_OK) return err;
    err = mcp2515_bit_modify(mcp, MCP_RXBnCTRL(mcp_rxb_base[0]),
        MCP_RXBnCTRL_RXM_MASK, MCP_RXBnCTRL_RXM(cfg->flags.rx_mode));
    if(err != ESP_OK) return err;

    err = mcp2515_bit_modify(mcp, MCP_RXBnSIDL(mcp_rxb_base[0]),
        MCP_EXIDE_MASK, MCP_EXIDE(cfg->flags.RX0_EXIDE));
    if(err != ESP_OK) return err;
    err = mcp2515_bit_modify(mcp, MCP_RXBnSIDL(mcp_rxb_base[1]),
        MCP_EXIDE_MASK, MCP_EXIDE(cfg->flags.RX1_EXIDE));
    if(err != ESP_OK) return err;

    return ESP_OK;
}

static mcp2515_id_t mcp2515_pack_id(uint32_t id, bool extended){
    mcp2515_id_t out = {0};

    if(!extended){
        out.sidh = (id >> 3) & MCP_SIDH_ID_MASK;
        out.sidl = (id & MCP_SIDL_ID_MASK) << 5;
        out.eid8 = 0;
        out.eid0 = 0;
    } else{
        out.sidh = (id >> 21) & MCP_SIDH_ID_MASK;

        out.sidl = ((id >> 16) & MCP_SIDL_EXT_ID_MASK1)
             | ((id >> 18) & MCP_SIDL_EXT_ID_MASK2)
             | MCP_EXIDE_MASK;

        out.eid8 = (id >> MCP_EID8_OFFSET) & MCP_EIDn_ID_MASK;
        out.eid0 = id & MCP_EIDn_ID_MASK;
    }

    return out;
}

static uint32_t mcp2515_unpack_id(mcp2515_id_t id, bool extended){
    uint32_t out = 0;

    if(!extended){
        out = ((uint32_t)(id.sidh << 3) | (id.sidl >> 5));
    } else {
        out =  ((uint32_t)id.sidh << 21) |
                     ((uint32_t)(id.sidl & MCP_SIDL_EXT_ID_MASK2) << 13) |
                     ((uint32_t)(id.sidl & MCP_SIDL_EXT_ID_MASK1) << 16) |
                     ((uint32_t)id.eid8 << MCP_EID8_OFFSET) |
                     id.eid0; 
    }

    return out;
}

static int8_t mcp2515_get_free_tx_buffer(mcp2515_handle_t* mcp) {
    uint8_t ctrl;
    for (int i = 0; i < 3; i++) {
        mcp2515_read_reg(mcp, mcp_txb_base[i], &ctrl);
        if (!(ctrl & MCP_TXBnTXREQ)) 
            return i;
    }
    return -1; 
}