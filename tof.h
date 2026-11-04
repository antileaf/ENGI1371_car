#pragma once

#include "base.h"
#include <math.h>

namespace tof {

// 读取一个已启动连续测距的传感器，距离单位统一为米。
// 读取会等待测量完成，最长等待时间由 setTimeout() 设置。
// 超时返回 false，并将距离设为 NAN；使用读数前必须检查返回值。
inline bool readDistance(VL53L0X &sensor, float &distanceMetres) {
	distanceMetres = NAN;

	const uint16_t distanceMm = sensor.readRangeContinuousMillimeters();

	if (sensor.timeoutOccurred())
		return false;

	distanceMetres = distanceMm / 1000.0f;
	return true;
}

// 按传感器 1、2 的顺序读取，不预设它们在车上的左右位置。
// 两个传感器都会被读取；仅当两次读取均未超时时返回 true。
// 单个传感器超时只会将其对应输出设为 NAN，另一个读数仍保留。
// 此接口只检查通信超时，不代表测距质量已通过验证。
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

bool setup_late() {
	// 先关闭全部传感器
	disableSensor(ports::XSHUT_1);
	disableSensor(ports::XSHUT_2);
	delay(10);

	Wire.begin();

	devices::tof1.setTimeout(500);
	devices::tof2.setTimeout(500);

	// 初始化传感器 1
	enableSensor(ports::XSHUT_1);

	const bool firstReady = devices::tof1.init();
	if (!firstReady) {
		Serial.println("Sensor 1 initialization failed!");
		// 隔离可能仍占用默认地址的设备，再检查第二个传感器。
		disableSensor(ports::XSHUT_1);
	}
	else
		devices::tof1.setAddress(0x30);

	// 初始化传感器 2
	enableSensor(ports::XSHUT_2);

	const bool secondReady = devices::tof2.init();
	if (!secondReady) {
		Serial.println("Sensor 2 initialization failed!");
		disableSensor(ports::XSHUT_2);
	}
	else
		devices::tof2.setAddress(0x31);

	// 开始连续测距
	if (!firstReady || !secondReady)
		return false;
	
	devices::tof1.startContinuous(50);
	devices::tof2.startContinuous(50);
	
	return true;
}

}
