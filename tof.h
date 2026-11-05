#pragma once

#include "base.h"
#include <math.h>

namespace tof {

// 读取一个已启动连续测距的传感器，距离单位统一为米。
// 读取会等待测量完成，最长等待时间由 setTimeout() 设置。
// 超时或总线传输失败时返回 false，距离设为 NAN。
inline bool readDistance(VL53L0X &sensor, float &distanceMetres) {
	distanceMetres = NAN;

	const uint16_t distanceMm = sensor.readRangeContinuousMillimeters();

	if (sensor.timeoutOccurred() || sensor.last_status != 0)
		return false;

	distanceMetres = distanceMm / 1000.0f;
	return true;
}

// 按传感器 1、2 的顺序读取，不预设它们在车上的左右位置。
// 两次通信均成功才返回 true。单侧失败保留另一侧读数。
// 此接口检查通信，不评估光学测距质量。
inline bool readDistances(float &distance1Metres, float &distance2Metres) {
	const bool valid1 = readDistance(devices::tof1, distance1Metres);
	const bool valid2 = readDistance(devices::tof2, distance2Metres);
	return valid1 && valid2;
}

// 关闭传感器：XSHUT 拉低
void disableSensor(int pin) {
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
}

// 启动传感器：释放 XSHUT，由外部电阻上拉
void enableSensor(int pin) {
	pinMode(pin, INPUT);
	delay(10);
}

inline void setup_early() {
	disableSensor(ports::XSHUT_1);
	disableSensor(ports::XSHUT_2);
}

#if defined(ARDUINO_ARCH_ESP32)
inline bool probeModel(int sensor, const char *stage, uint8_t address) {
	uint8_t model = 0;
	const esp_err_t err = i2c.readRegister(address, VL53L0X::IDENTIFICATION_MODEL_ID, &model, 1);
	Serial.printf("[ToF %d] %s addr=0x%02X I2C=%s", sensor, stage, address, esp_err_to_name(err));
	if (err == ESP_OK) Serial.printf(" model=0x%02X (expected 0xEE)", model);
	Serial.println();
	return err == ESP_OK && model == 0xEE;
}
#endif

inline bool bringUp(VL53L0X &sensor, int number, int xshut, uint8_t address) {
	enableSensor(xshut);
#if defined(ARDUINO_ARCH_ESP32)
	Serial.printf("[ToF %d] released XSHUT GPIO%d level=%d\n", number, xshut, digitalRead(xshut));
	if (!probeModel(number, "boot", 0x29)) return false;
#endif
	if (!sensor.init()) {
#if defined(ARDUINO_ARCH_ESP32)
		Serial.printf("[ToF %d] init failed at %s I2C=%s\n", number, sensor.initStage, esp_err_to_name(sensor.last_status));
#else
		Serial.print("ToF init failed: "); Serial.println(number);
#endif
		return false;
	}
	sensor.setAddress(address);
#if defined(ARDUINO_ARCH_ESP32)
	Serial.printf("[ToF %d] setAddress(0x%02X) I2C=%s\n", number, address, esp_err_to_name(sensor.last_status));
	return sensor.last_status == ESP_OK && probeModel(number, "verify address", address);
#else
	return sensor.last_status == 0;
#endif
}

static bool firstInitialized = false, secondInitialized = false;

bool setup_late() {
	firstInitialized = secondInitialized = false;
	delay(10);

	// 两颗传感器共用总线，通过独立 XSHUT 依次分配地址。
#if defined(ARDUINO_ARCH_ESP32)
	const esp_err_t bus = i2c.init(ports::TOF_SCL, ports::TOF_SDA);
	Serial.printf("[ToF] I2C SDA=%d SCL=%d 100kHz init=%s\n", ports::TOF_SDA, ports::TOF_SCL, esp_err_to_name(bus));
	if (bus != ESP_OK) {
		return false;
	}
	if (probeModel(0, "both XSHUT low; expect no reply", 0x29))
		Serial.println("[ToF] Unexpected VL53L0X at 0x29; check both XSHUT connections");
#else
	Wire.begin();
#endif

	devices::tof1.setTimeout(500);
	devices::tof2.setTimeout(500);

	const bool firstReady = bringUp(devices::tof1, 1, ports::XSHUT_1, 0x30);
	if (!firstReady) {
		disableSensor(ports::XSHUT_1);
		delay(10);
	}
	const bool secondReady = bringUp(devices::tof2, 2, ports::XSHUT_2, 0x31);
	if (!secondReady) disableSensor(ports::XSHUT_2);

	// 开始连续测距
	firstInitialized = firstReady;
	secondInitialized = secondReady;
	if (!firstReady || !secondReady)
		return false;
	
	devices::tof1.startContinuous(50);
	devices::tof2.startContinuous(50);
	
#if defined(ARDUINO_ARCH_ESP32)
	Serial.printf("[ToF] startContinuous: 1=%s 2=%s\n", esp_err_to_name(devices::tof1.last_status), esp_err_to_name(devices::tof2.last_status));
#endif
	firstInitialized = devices::tof1.last_status == 0;
	secondInitialized = devices::tof2.last_status == 0;
	return firstInitialized && secondInitialized;
}

}
