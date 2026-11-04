#pragma once

#include "base.h"

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
	digitalWrite(ports::SERVO, LOW);
}

// 仅显式重新启动控制器时恢复脉冲，先设置目标以避免旧目标被重新输出。
inline void enable(int angle) {
	servo.write(angle);
	
	if (!servo.attached())
		servo.attach(ports::SERVO);
}

}
