#pragma once

#include "tof.h"
#include "ultrasonic.h"
#include "motor.h"
#include "servo.h"
#include "status_led.h"
#include "serial_commands.h"

namespace hardware_test {

constexpr int TEST_PWM = 180; // 沿用原电机测试的 PWM
constexpr int SERVO_ANGLE = static_cast<int>(calibration::SERVO_CENTRE_DEG + 0.5f);
constexpr uint32_t OUTPUT_PERIOD_MS = 250;
static uint32_t lastOutputMs = 0;
constexpr uint32_t MOTOR_REFRESH_PERIOD_MS = 3000;
static uint32_t lastMotorRefreshMs = 0;
static int pwmA = 0, pwmB = 0;

inline void applyOutputs() {
	pwmA = TEST_PWM * calibration::MOTOR_A_DIRECTION;
	pwmB = TEST_PWM * calibration::MOTOR_B_DIRECTION;

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
	Serial.println(F("Send servo angle 0-180 or boot, followed by CR or LF."));

	lastOutputMs = static_cast<uint32_t>(millis());

	Serial.println(F("Hardware test: ToF/front=m, open-loop PWM"));
	
	applyOutputs();
	lastMotorRefreshMs = static_cast<uint32_t>(millis());
}

inline void printDistance(float value, bool valid) {
	if (valid && isfinite(value)) Serial.print(value, 3);
	else Serial.print(F("TIMEOUT"));
}

inline void readServoCommand() {
	serial_commands::update(true);
}

// 连续固定 PWM 前进测试，测距仍有短暂阻塞；此模式不执行避障。
inline void update() {
	readServoCommand();
	// 每 3 秒重新设置电机方向和 PWM，使用独立计时，不阻塞其它测试。
	const uint32_t motorNow = static_cast<uint32_t>(millis());
	if (motorNow - lastMotorRefreshMs >= MOTOR_REFRESH_PERIOD_MS) {
		applyOutputs();
		lastMotorRefreshMs = motorNow;
	}
	if (tof::recoveryDue()) {
		tof::retryFailedSensors();
	}
	const uint32_t now = static_cast<uint32_t>(millis());
	if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
	lastOutputMs = now;
	float distance1 = NAN, distance2 = NAN, front;
	const bool valid1 = tof::firstInitialized && tof::readDistance(devices::tof1, distance1);
	const bool valid2 = tof::secondInitialized && tof::readDistance(devices::tof2, distance2);
	// 距离过远不表示传感器不可用，不点亮故障灯。
	status_led::update(!valid1 || !isfinite(distance1) || distance1 <= 0,
		!valid2 || !isfinite(distance2) || distance2 <= 0);
	const bool validFront = ultrasonic::readDistance(front, 12000UL);

	Serial.print(F("t_ms=")); Serial.print(now);
	Serial.print(F(" tof1="));
	if (tof::firstInitialized) printDistance(distance1, valid1);
	else Serial.print(F("INIT_FAILED"));
	if (valid1 && distance1 > calibration::TOF_MAX_DISTANCE_M) Serial.print(F("(OUT_OF_RANGE)"));
	Serial.print(F(" tof2="));
	if (tof::secondInitialized) printDistance(distance2, valid2);
	else Serial.print(F("INIT_FAILED"));
	if (valid2 && distance2 > calibration::TOF_MAX_DISTANCE_M) Serial.print(F("(OUT_OF_RANGE)"));
	Serial.print(F(" front=")); printDistance(front, validFront);
	Serial.println();
}

} // namespace hardware_test
