#pragma once

#include "base.h"

namespace motor {

void setMotor(int enablePin, int in1, int in2, int speed) {
	speed = constrain(speed, -255, 255);

	if (speed == 0) {
		// 关闭对应桥臂，让车轮惯性滑停。
		digitalWrite(enablePin, LOW);
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
		return;
	}

	analogWrite(enablePin, 0);

	digitalWrite(in1, speed > 0);
	digitalWrite(in2, speed < 0);

	analogWrite(enablePin, abs(speed));
}

void motorA(int speed) {
	setMotor(ports::ENA, ports::IN_1, ports::IN_2, speed);
}

void motorB(int speed) {
	setMotor(ports::ENB, ports::IN_3, ports::IN_4, speed);
}

void stopMotors() {
	motorA(0);
	motorB(0);
}

void brakeMotors() {
	// 两个方向脚已由 stopMotors() 拉低，再使能以执行电气制动。
	stopMotors();
	digitalWrite(ports::ENA, HIGH);
	digitalWrite(ports::ENB, HIGH);
}

void setup_early() {
	pinMode(ports::IN_1, OUTPUT);
	pinMode(ports::IN_3, OUTPUT);
	pinMode(ports::IN_2, OUTPUT);
	pinMode(ports::IN_4, OUTPUT);

	pinMode(ports::ENA, OUTPUT);
	pinMode(ports::ENB, OUTPUT);
}

void setup_late() {
	stopMotors();
}

} // namespace motor
