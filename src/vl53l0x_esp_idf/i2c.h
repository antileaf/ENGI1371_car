#pragma once
#include <driver/i2c.h>

class _i2c {
   public:
    esp_err_t init(int scl = 22, int sda = 21);
    esp_err_t write(uint8_t addr, const uint8_t* data, size_t size);
    esp_err_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data, size_t size);

   private:
    i2c_port_t i2c_master_port = I2C_NUM_0;
};

extern _i2c i2c;
