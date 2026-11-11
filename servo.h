#pragma once

#include "base.h"
#include <esp32-hal-matrix.h>

namespace servo {

using devices::servo;

void setup_early() {
	pinMode(ports::SERVO, OUTPUT);
}

void setup_late() {
	servo.attach(ports::SERVO);

	// 初始化到中间位置
	servo.write(static_cast<int>(calibration::SERVO_CENTRE_DEG + 0.5f));
}

void write(int angle) {
	servo.write(angle);
}

// 停止控制脉冲，不改变当前目标角；这不是切断舵机电源。
inline void disable() {
	servo.detach();
	// Disconnect the MCPWM output before driving GPIO low.
	pinMatrixOutDetach(ports::SERVO, false, false);
	pinMode(ports::SERVO, OUTPUT);
	digitalWrite(ports::SERVO, LOW);
}

inline void enable(int angle) {
	if (!servo.attached())
		servo.attach(ports::SERVO);
	servo.write(angle);
}

}
