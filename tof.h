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
	digitalWrite(pin, LOW);
	pinMode(pin, OUTPUT);
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

bool setup_late() {
	delay(10);

	// 两颗传感器共用总线，通过独立 XSHUT 依次分配地址。
#if defined(ARDUINO_ARCH_ESP32)
	if (i2c.init(ports::TOF_SCL, ports::TOF_SDA) != ESP_OK) {
		Serial.println("ToF I2C initialization failed!");
		return false;
	}
#else
	Wire.begin();
#endif

	devices::tof1.setTimeout(500);
	devices::tof2.setTimeout(500);

	// 初始化传感器 1
	enableSensor(ports::XSHUT_1);

	bool firstReady = devices::tof1.init();
	if (firstReady) {
		devices::tof1.setAddress(0x30);
		firstReady = devices::tof1.last_status == 0;
	}
	if (!firstReady) {
		Serial.println("Sensor 1 initialization failed!");
		// 隔离可能仍占用默认地址的设备，再检查第二个传感器。
		disableSensor(ports::XSHUT_1);
	}

	// 初始化传感器 2
	enableSensor(ports::XSHUT_2);

	bool secondReady = devices::tof2.init();
	if (secondReady) {
		devices::tof2.setAddress(0x31);
		secondReady = devices::tof2.last_status == 0;
	}
	if (!secondReady) {
		Serial.println("Sensor 2 initialization failed!");
		disableSensor(ports::XSHUT_2);
	}

	// 开始连续测距
	if (!firstReady || !secondReady)
		return false;
	
	devices::tof1.startContinuous(50);
	devices::tof2.startContinuous(50);
	
	return devices::tof1.last_status == 0 && devices::tof2.last_status == 0;
}

}
