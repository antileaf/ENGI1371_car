#pragma once

#include "tof.h"
#include "ultrasonic.h"
#include "motor.h"
#include "servo.h"

namespace hardware_test {

constexpr int TEST_PWM = 50; // 沿用原电机测试的 PWM
constexpr int SERVO_ANGLE = 75; // 保留当前舵机测试角度，可在此调整
constexpr uint32_t OUTPUT_PERIOD_MS = 250;
static uint32_t lastOutputMs = 0;
static int pwmA = 0, pwmB = 0;

inline void applyOutputs() {
	pwmA = TEST_PWM;
	pwmB = -TEST_PWM;

	motor::motorA(pwmA);
	motor::motorB(pwmB);

	Serial.print(F(" PWM_A=")); Serial.print(pwmA);
	Serial.print(F(" PWM_B=")); Serial.println(pwmB);
}

// 在底层初始化后调用一次；这是开环硬件测试，不运行道路控制或避障。
inline void setup() {
	motor::stopMotors();

	devices::tof1.setTimeout(25);
	devices::tof2.setTimeout(25);

	servo::write(SERVO_ANGLE);


	lastOutputMs = static_cast<uint32_t>(millis());

	Serial.println(F("Hardware test: ToF/front=m, open-loop PWM"));
	
	applyOutputs();
}

inline void printDistance(float value, bool valid) {
	if (valid && isfinite(value)) Serial.print(value, 3);
	else Serial.print(F("TIMEOUT"));
}

// 连续固定 PWM 前进测试，测距仍有短暂阻塞；此模式不执行避障。
inline void update() {
	const uint32_t now = static_cast<uint32_t>(millis());
	if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
	lastOutputMs = now;
	float distance1, distance2, front;
	const bool valid1 = tof::readDistance(devices::tof1, distance1);
	const bool valid2 = tof::readDistance(devices::tof2, distance2);
	const bool validFront = ultrasonic::readDistance(front, 12000UL);

	Serial.print(F("t_ms=")); Serial.print(now);
	Serial.print(F(" tof1=")); printDistance(distance1, valid1);
	Serial.print(F(" tof2=")); printDistance(distance2, valid2);
	Serial.print(F(" front=")); printDistance(front, validFront);
	Serial.println();
}

} // hardware_test 命名空间结束
