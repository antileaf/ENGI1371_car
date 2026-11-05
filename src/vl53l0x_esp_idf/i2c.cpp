// Upstream: hirobon1690/vl53l0x-esp-idf @ 5c77c163d27d5bff4b5118508534fbdbfdffe318
#if defined(ARDUINO_ARCH_ESP32)
#include "i2c.h"
#include <driver/i2c.h>

_i2c i2c;

// Allow scheduler jitter; keep bus failure detection bounded.
static const TickType_t transferTimeout = pdMS_TO_TICKS(5) + 1;

esp_err_t _i2c::init(int scl, int sda) {
    i2c_config_t conf = {};
    const int I2C_MASTER_SCL_IO = scl;       /*!< GPIO number used for I2C master clock */
    const int I2C_MASTER_SDA_IO = sda;       /*!< GPIO number used for I2C master data  */
    const int I2C_MASTER_FREQ_HZ = 100000;   /*!< I2C master clock frequency */
    const int I2C_MASTER_TX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    const int I2C_MASTER_RX_BUF_DISABLE = 0; /*!< I2C master doesn't need buffer */
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;  // select GPIO specific to your project
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;  // select GPIO specific to your project
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;  // select frequency specific to your project
    conf.clk_flags = 0;
    esp_err_t result = i2c_param_config(i2c_master_port, &conf);
    if (result != ESP_OK) return result;
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t _i2c::write(uint8_t addr, const uint8_t* data, size_t size) {
    return i2c_master_write_to_device(i2c_master_port, addr, data, size, transferTimeout);
}

esp_err_t _i2c::readRegister(uint8_t addr, uint8_t reg, uint8_t* data, size_t size) {
    // IDF uses a repeated START and NACKs the final byte.
    return i2c_master_write_read_device(i2c_master_port, addr, &reg, 1, data, size, transferTimeout);
}
#endif
