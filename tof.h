#pragma once

#include "base.h"
#include <math.h>

namespace tof {
static bool firstInitialized = false, secondInitialized = false;
static bool busInitialized = false;
constexpr uint8_t READ_FAILURE_LIMIT = 5;
constexpr uint32_t RECOVERY_PERIOD_MS = 2000;
static uint8_t firstFailures = 0, secondFailures = 0;
static uint32_t recoveryAtMs = 0;

inline void recordReadResult(VL53L0X &sensor, bool valid) {
	const bool first = &sensor == &devices::tof1;
	if (!first && &sensor != &devices::tof2) return;
	uint8_t &failures = first ? firstFailures : secondFailures;
	bool &initialized = first ? firstInitialized : secondInitialized;
	if (valid) { failures = 0; return; }
	if (!initialized) return;
	if (++failures < READ_FAILURE_LIMIT) return;
	initialized = false;
	// 先关闭故障侧，避免其意外回到默认地址后与另一侧重初始化冲突。
	const int pin = first ? ports::XSHUT_1 : ports::XSHUT_2;
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	recoveryAtMs = static_cast<uint32_t>(millis());
	Serial.printf("[ToF %d] consecutive read failures; scheduling recovery\n", first ? 1 : 2);
}

inline bool recoveryDue() {
	return (!firstInitialized || !secondInitialized) &&
		static_cast<uint32_t>(millis()) - recoveryAtMs >= RECOVERY_PERIOD_MS;
}

// 读取一个已启动连续测距的传感器，距离单位统一为米。
// 读取会等待测量完成，最长等待时间由 setTimeout() 设置。
// 超时或总线传输失败时返回 false，距离设为 NAN。
inline bool readDistance(VL53L0X &sensor, float &distanceMetres) {
	distanceMetres = NAN;

	const uint16_t distanceMm = sensor.readRangeContinuousMillimeters();

	const bool valid = !sensor.timeoutOccurred() && sensor.last_status == ESP_OK;
	recordReadResult(sensor, valid);
	if (!valid)
		return false;

	distanceMetres = distanceMm / 1000.0f;
	return true;
}

// 按传感器 1、2 的顺序读取，不预设它们在车上的左右位置。
// 两次通信均成功才返回 true。单侧失败保留另一侧读数。
// 此接口检查通信，不评估光学测距质量。
inline bool readDistances(float &distance1Metres, float &distance2Metres) {
	distance1Metres = distance2Metres = NAN;
	const bool valid1 = firstInitialized && readDistance(devices::tof1, distance1Metres);
	const bool valid2 = secondInitialized && readDistance(devices::tof2, distance2Metres);
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

inline bool probeModel(int sensor, const char *stage, uint8_t address) {
	uint8_t model = 0;
	const esp_err_t err = i2c.readRegister(address, VL53L0X::IDENTIFICATION_MODEL_ID, &model, 1);
	Serial.printf("[ToF %d] %s addr=0x%02X I2C=%s", sensor, stage, address, esp_err_to_name(err));
	if (err == ESP_OK) Serial.printf(" model=0x%02X (expected 0xEE)", model);
	Serial.println();
	return err == ESP_OK && model == 0xEE;
}

inline bool bringUp(VL53L0X &sensor, int number, int xshut, uint8_t address) {
	enableSensor(xshut);
	Serial.printf("[ToF %d] released XSHUT GPIO%d level=%d\n", number, xshut, digitalRead(xshut));
	if (!probeModel(number, "boot", 0x29)) return false;
	if (!sensor.init()) {
		Serial.printf("[ToF %d] init failed at %s I2C=%s\n", number, sensor.initStage, esp_err_to_name(sensor.last_status));
		return false;
	}
	sensor.setAddress(address);
	Serial.printf("[ToF %d] setAddress(0x%02X) I2C=%s\n", number, address, esp_err_to_name(sensor.last_status));
	return sensor.last_status == ESP_OK && probeModel(number, "verify address", address);
}

bool setup_late() {
	firstInitialized = secondInitialized = false;
	firstFailures = secondFailures = 0;
	recoveryAtMs = static_cast<uint32_t>(millis());
	// XSHUT 复位后地址回到默认值，同步清除对象保存的旧地址和错误状态。
	devices::tof1 = VL53L0X();
	devices::tof2 = VL53L0X();
	delay(10);

	// 两颗传感器共用总线，通过独立 XSHUT 依次分配地址。
	const esp_err_t bus = i2c.init(ports::TOF_SCL, ports::TOF_SDA);
	busInitialized = bus == ESP_OK;
	Serial.printf("[ToF] I2C SDA=%d SCL=%d 100kHz init=%s\n", ports::TOF_SDA, ports::TOF_SCL, esp_err_to_name(bus));
	if (bus != ESP_OK) {
		return false;
	}
	if (probeModel(0, "both XSHUT low; expect no reply", 0x29))
		Serial.println("[ToF] Unexpected VL53L0X at 0x29; check both XSHUT connections");

	devices::tof1.setTimeout(500);
	devices::tof2.setTimeout(500);

	const bool firstReady = bringUp(devices::tof1, 1, ports::XSHUT_1, 0x30);
	if (!firstReady) {
		disableSensor(ports::XSHUT_1);
		delay(10);
	}
	const bool secondReady = bringUp(devices::tof2, 2, ports::XSHUT_2, 0x31);
	if (!secondReady) disableSensor(ports::XSHUT_2);

	// 各通道独立启动，硬件测试可继续使用成功的传感器。
	if (firstReady)
		devices::tof1.startContinuous(50);
	if (secondReady)
		devices::tof2.startContinuous(50);
	
	Serial.printf("[ToF] startContinuous: 1=%s 2=%s\n", esp_err_to_name(devices::tof1.last_status), esp_err_to_name(devices::tof2.last_status));
	firstInitialized = firstReady && devices::tof1.last_status == 0;
	secondInitialized = secondReady && devices::tof2.last_status == 0;
	if (!firstInitialized) disableSensor(ports::XSHUT_1);
	if (!secondInitialized) disableSensor(ports::XSHUT_2);
	return firstInitialized && secondInitialized;
}

inline bool retrySensor(VL53L0X &sensor, int number, int xshut, uint8_t address) {
	disableSensor(xshut);
	delay(10);
	sensor = VL53L0X();
	sensor.setTimeout(500);
	bool ready = bringUp(sensor, number, xshut, address);
	if (ready) {
		sensor.startContinuous(50);
		ready = sensor.last_status == ESP_OK;
	}
	if (ready)
		sensor.setTimeout(25);
	else
		disableSensor(xshut);
	Serial.printf("[ToF %d] retry %s\n", number, ready ? "succeeded" : "failed");
	return ready;
}

// 已成功的通道保持原地址和连续测距状态；调用方负责控制重试间隔。
inline void retryFailedSensors() {
	if (firstInitialized && secondInitialized) return;
	if (!busInitialized) {
		const esp_err_t bus = i2c.init(ports::TOF_SCL, ports::TOF_SDA);
		busInitialized = bus == ESP_OK;
		if (!busInitialized) {
			recoveryAtMs = static_cast<uint32_t>(millis());
			Serial.printf("[ToF] I2C retry failed: %s\n", esp_err_to_name(bus));
			return;
		}
	}
	if (!firstInitialized) {
		firstInitialized = retrySensor(devices::tof1, 1, ports::XSHUT_1, 0x30);
		if (firstInitialized) firstFailures = 0;
	}
	if (!secondInitialized) {
		secondInitialized = retrySensor(devices::tof2, 2, ports::XSHUT_2, 0x31);
		if (secondInitialized) secondFailures = 0;
	}
	recoveryAtMs = static_cast<uint32_t>(millis());
}

}
