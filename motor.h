#pragma once

#include "base.h"

namespace motor {

void setMotor(int pwm, int in1, int in2, int speed) {
	speed = constrain(speed, -255, 255);

	// if (brake) {
	//     digitalWrite(in1, HIGH);
	//     digitalWrite(in2, HIGH);
	//     // digitalWrite(pwm, LOW); // 任意
	//     return;
	// }

	if (speed == 0) {
		digitalWrite(in1, LOW);
		digitalWrite(in2, LOW);
		digitalWrite(pwm, HIGH);
		return;
	}

	analogWrite(pwm, 0);

	digitalWrite(in1, speed > 0);
	digitalWrite(in2, speed < 0);

	analogWrite(pwm, abs(speed));
}

void motorA(int speed) {
	setMotor(ports::PWMA, ports::AIN_1, ports::AIN_2, speed);
}

void motorB(int speed) {
	setMotor(ports::PWMB, ports::BIN_1, ports::BIN_2, speed);
}

void stopMotors() {
	motorA(0);
	motorB(0);
}

void brakeMotors() { // TODO
	motorA(0);
	motorB(0);
}

void setup_early() {
	pinMode(ports::AIN_1, OUTPUT);
	pinMode(ports::BIN_1, OUTPUT);
	pinMode(ports::AIN_2, OUTPUT);
	pinMode(ports::BIN_2, OUTPUT);

	pinMode(ports::STBY, OUTPUT);

	pinMode(ports::XSHUT_1, OUTPUT);
	// pinMode(SDA_1, INPUT);
	// pinMode(SCL_1, OUTPUT);

	pinMode(ports::XSHUT_2, OUTPUT);
	// pinMode(SDA_2, INPUT);
	// pinMode(SCL_2, OUTPUT);

	pinMode(ports::PWMA, OUTPUT);
	pinMode(ports::PWMB, OUTPUT);
}

void setup_late() {
	// 先让驱动板进入待机
	pinMode(ports::STBY, OUTPUT);
	digitalWrite(ports::STBY, LOW);

	pinMode(ports::PWMA, OUTPUT);
	pinMode(ports::PWMB, OUTPUT);

	pinMode(ports::AIN_1, OUTPUT);
	pinMode(ports::AIN_2, OUTPUT);
	pinMode(ports::BIN_1, OUTPUT);
	pinMode(ports::BIN_2, OUTPUT);

	stopMotors();

	delay(500);

	// 启用驱动板
	digitalWrite(ports::STBY, HIGH);
}

} // namespace motor
