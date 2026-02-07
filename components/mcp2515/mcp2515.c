#include "mcp2515.h"

#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_OPMOD_MASK  0xE0
#define MCP_OPMOD_SLEEP 0x20

#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2A

#define MCP_CANINTE         0x2B
#define MCP_CANINTE_WAKIE   (1 << 6)
#define MCP_CANINTF         0x2C
#define MCP_EFLG            0x2D
#define MCP_TEC 0x1C

// TX buffers
#define MCP_TXB0_BASE 0x30
#define MCP_TXB1_BASE 0x40
#define MCP_TXB2_BASE 0x50
#define MCP_TXB_TXREQ (1 << 3)
#define MCP_TXB_TXERR (1 << 4)
#define MCP_TXB_MLOA  (1 << 5)
// RX buffers
#define MCP_RXB0CTRL 0x60
#define MCP_RXB1CTRL 0x70

// REVIEW LATER
#define MCP_TXB_CTRL(base) ((base) + 0x00)
#define MCP_TXB_SIDH(base) ((base) + 0x01)
#define MCP_TXB_SIDL(base) ((base) + 0x02)
#define MCP_TXB_EID8(base) ((base) + 0x03)
#define MCP_TXB_EID0(base) ((base) + 0x04)
#define MCP_TXB_DLC(base) ((base) + 0x05)
#define MCP_TXB_D0(base) ((base) + 0x06)

// FOR NOW I ONLY IMPLEMENT STANDARD ID
#define MCP_SIDH_ID_MASK 0xFF
#define MCP_SIDL_ID_MASK 0x07

#define MCP_CMD_RESET       0xC0
#define MCP_CMD_READ        0x03
#define MCP_CMD_WRITE       0x02
#define MCP_CMD_READ_STATUS 0xA0
#define MCP_CMD_RX_STATUS   0xB0
#define MCP_CMD_BIT_MODIFY  0x05

static esp_err_t mcp2515_bit_modify(mcp2515_handle_t* mcp, uint8_t reg,
                                    uint8_t mask, uint8_t data);
static bool mcp2515_is_awake(mcp2515_handle_t* mcp);

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

    return mcp2515_reset(mcp);
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

esp_err_t mcp2515_read_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t* val) {
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

esp_err_t mcp2515_write_reg(mcp2515_handle_t* mcp, uint8_t reg, uint8_t val) {
    if (!mcp || !mcp->spi)
        return ESP_ERR_INVALID_ARG;

    uint8_t tx_buf[3] = {MCP_CMD_WRITE, reg, val};
    spi_transaction_t t = {.length = 8 * sizeof(tx_buf), .tx_buffer = tx_buf};

    return spi_device_transmit(mcp->spi, &t);
}

esp_err_t mcp2515_bit_modify(mcp2515_handle_t* mcp, uint8_t reg, uint8_t mask,
                             uint8_t data) {
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

esp_err_t mcp2515_transmit(mcp2515_handle_t* mcp, uint16_t id, uint8_t dlc, const uint8_t* data){
    if (dlc > 8) return ESP_ERR_INVALID_ARG;

    uint8_t sidh = (id >> 3) & MCP_SIDH_ID_MASK;
    uint8_t sidl = (id & MCP_SIDL_ID_MASK) << 5;

    mcp2515_write_reg(mcp, MCP_TXB_SIDH(MCP_TXB0_BASE), sidh);
    mcp2515_write_reg(mcp, MCP_TXB_SIDL(MCP_TXB0_BASE), sidl);
    mcp2515_write_reg(mcp, MCP_TXB_EID8(MCP_TXB0_BASE), 0);
    mcp2515_write_reg(mcp, MCP_TXB_EID0(MCP_TXB0_BASE), 0);
    mcp2515_write_reg(mcp, MCP_TXB_DLC(MCP_TXB0_BASE), dlc);

    for (int i = 0;i < dlc; i++){
        mcp2515_write_reg(mcp, MCP_TXB_D0(MCP_TXB0_BASE) + i, data[i]);
    }

    mcp2515_bit_modify(mcp, MCP_TXB0_BASE, MCP_TXB_TXREQ, MCP_TXB_TXREQ);

    uint8_t ctrl;

    do {
        mcp2515_read_reg(mcp, MCP_TXB0_BASE, &ctrl);
    } while (ctrl & MCP_TXB_TXREQ);

    if (ctrl & MCP_TXB_TXERR) 
        return ESP_FAIL;
    
    return ESP_OK;
}