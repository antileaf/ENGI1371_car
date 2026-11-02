#pragma once

#include <math.h>

#include "base.h"

namespace ultrasonic {

// 回波等待上限，单位：微秒。
constexpr unsigned long ECHO_TIMEOUT_US = 50000UL; // 50ms

// 发出一次触发脉冲并读取距离，距离单位统一为米。
// 此接口会阻塞等待回波；调用方应按传感器要求安排触发间隔。
// 未收到完整回波时返回 false，并将距离设为 NAN。
inline bool readDistance(float &distanceMetres,
						 unsigned long timeoutUs = ECHO_TIMEOUT_US) {
	distanceMetres = NAN;

	if (timeoutUs == 0)
		return false;

	digitalWrite(ports::ULTRASONIC_TRIG, LOW);
	delayMicroseconds(2);
	digitalWrite(ports::ULTRASONIC_TRIG, HIGH);
	delayMicroseconds(10);
	digitalWrite(ports::ULTRASONIC_TRIG, LOW);

	const unsigned long duration = pulseIn(ports::ULTRASONIC_ECHO, HIGH, timeoutUs);
	
	if (duration == 0)
		return false;

	// 测试代码以 duration / 58 得到厘米，再除以 100 转为米。
	distanceMetres = duration / 5800.0f;
	return true;
}

void setup_early() {
	pinMode(ports::ULTRASONIC_TRIG, OUTPUT);
	pinMode(ports::ULTRASONIC_ECHO, INPUT);
}

void setup_late() {
	digitalWrite(ports::ULTRASONIC_TRIG, LOW);
}

}
