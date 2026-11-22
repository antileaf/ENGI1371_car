#pragma once
#include "Arduino.h"
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
inline const char *esp_err_to_name(esp_err_t error) { return error ? "ERROR" : "ESP_OK"; }
struct MockI2C {
 esp_err_t init(int, int) { return ESP_OK; }
 esp_err_t readRegister(uint8_t, uint8_t, uint8_t *data, size_t) { *data = 0xEE; return ESP_OK; }
} i2c;
static bool mockTofInitFails = false;
struct VL53L0X{
 static constexpr uint8_t IDENTIFICATION_MODEL_ID = 0xC0;
 const char *initStage = "mock";
 int last_status=0;
 uint16_t mm=300; bool timeout=false; uint32_t waitUs=0;
 uint16_t readRangeContinuousMillimeters(){nowUs+=waitUs;return mm;}
 bool timeoutOccurred(){return timeout;}
 void setTimeout(int){} bool init(){return !mockTofInitFails;}
 void setAddress(int){} void startContinuous(int){}
};
