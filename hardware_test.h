#pragma once

#include "tof.h"
#include "ultrasonic.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"

namespace hardware_test {

constexpr int TEST_PWM = 50; // 沿用原电机测试的 PWM
constexpr int SERVO_ANGLE = static_cast<int>(calibration::SERVO_CENTRE_DEG); // 保留当前舵机测试角度，可在此调整
constexpr uint32_t OUTPUT_PERIOD_MS = 250;
constexpr uint32_t RUN_MS = 5000, PAUSE_MS = 3000, CYCLE_PAUSE_MS = 10000;
// 阶段：A 正转、停、A 反转、停、B 正转、停、B 反转、停、双电机正转、停。
static uint8_t phase = 0;
static uint32_t phaseSinceMs = 0, lastOutputMs = 0;
static encoder::Counts initialCounts = {}, previousCounts = {};
static int pwmA = 0, pwmB = 0;

inline void applyPhase() {
	// pwmA = phase == 0 || phase == 8 ? TEST_PWM : phase == 2 ? -TEST_PWM : 0;
	// pwmB = phase == 4 || phase == 8 ? TEST_PWM : phase == 6 ? -TEST_PWM : 0;

	pwmA = -TEST_PWM;
	pwmB = TEST_PWM;

	motor::motorA(pwmA);
	motor::motorB(pwmB);

	Serial.print(F("Test phase=")); Serial.print(phase);
	Serial.print(F(" PWM_A=")); Serial.print(pwmA);
	Serial.print(F(" PWM_B=")); Serial.println(pwmB);
}

// 在底层初始化后调用一次；这是开环硬件测试，不运行道路控制或避障。
inline void setup() {
	motor::stopMotors();

	devices::tof1.setTimeout(25);
	devices::tof2.setTimeout(25);

	servo::write(SERVO_ANGLE);
	Serial.println(F("Servo: send 0-180 degrees followed by CR or LF."));

	initialCounts = previousCounts = encoder::readCounts();
	encoder::resetSpeedWindow();

	phaseSinceMs = lastOutputMs = static_cast<uint32_t>(millis());
	phase = 0;

	Serial.println(F("Hardware test: ToF/front=m, encoder counts=raw, speed=m/s"));
	Serial.println(F("Encoder1=A, Encoder2=B; speed scaling depends on calibration."));
	
	applyPhase();
}

inline void advancePhase() {
	const uint32_t now = static_cast<uint32_t>(millis());
	const uint32_t duration = phase == 9 ? CYCLE_PAUSE_MS : (phase % 2 ? PAUSE_MS : RUN_MS);
	if (now - phaseSinceMs < duration) return;
	phase = (phase + 1) % 10;
	phaseSinceMs = now;
	applyPhase();
}

inline void printDistance(float value, bool valid) {
	if (valid && isfinite(value)) Serial.print(value, 3);
	else Serial.print(F("TIMEOUT"));
}

// 不再使用长 delay/getchar，电机运行和停车期间都持续采样输出。
// 测距接口自身仍有短暂阻塞，阶段切换时间会有相应误差。
inline void readServoCommand() {
	// 分帧接收十进制角度，遇到行尾才执行；错误行整行丢弃。
	static int angle = 0;
	static bool hasDigits = false, invalid = false;
	while (Serial.available() > 0) {
		const int c = Serial.read();
		if (c == '\r' || c == '\n') {
			if (invalid)
				Serial.println(F("Invalid servo angle (expected 0-180)."));
			else if (hasDigits) {
				servo::write(angle);
				Serial.print(F("servo_deg="));
				Serial.println(angle);
			}
			angle = 0;
			hasDigits = invalid = false;
		}
		else if (!invalid && c >= '0' && c <= '9') {
			hasDigits = true;
			angle = angle * 10 + (c - '0');
			invalid = angle > 180;
		}
		else
			invalid = true;
	}
}

inline void update() {
	readServoCommand();
	advancePhase();
	const uint32_t now = static_cast<uint32_t>(millis());
	if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
	lastOutputMs = now;
	float distance1, distance2, front, speedMotorA, speedMotorB;
	const bool valid1 = tof::readDistance(devices::tof1, distance1);
	const bool valid2 = tof::readDistance(devices::tof2, distance2);
	const bool validFront = ultrasonic::readDistance(front, 12000UL);
	const encoder::Counts counts = encoder::readCounts();
	const bool validSpeed = encoder::readSpeeds(speedMotorA, speedMotorB);

	Serial.print(F("t_ms=")); Serial.print(now);
	Serial.print(F(" phase=")); Serial.print(phase);
	Serial.print(F(" tof1=")); printDistance(distance1, valid1);
	Serial.print(F(" tof2=")); printDistance(distance2, valid2);
	Serial.print(F(" front=")); printDistance(front, validFront);
	// 原始有符号累计变化量，未应用方向修正；B 电机前进时原始计数为负。
	// 下方 v_motor_a/v_motor_b 已应用方向修正，车身前进时均应为正。
	Serial.print(F(" encA=")); Serial.print(encoder::countDelta(counts.motorA, initialCounts.motorA));
	Serial.print(F(" encB=")); Serial.print(encoder::countDelta(counts.motorB, initialCounts.motorB));
	Serial.print(F(" dA=")); Serial.print(encoder::countDelta(counts.motorA, previousCounts.motorA));
	Serial.print(F(" dB=")); Serial.print(encoder::countDelta(counts.motorB, previousCounts.motorB));
	Serial.print(F(" v_motor_a="));
	if (validSpeed) Serial.print(speedMotorA, 4); else Serial.print(F("NA"));
	Serial.print(F(" v_motor_b="));
	if (validSpeed) Serial.print(speedMotorB, 4); else Serial.print(F("NA"));
	Serial.println();
	previousCounts = counts;
	advancePhase();
}

} // hardware_test 命名空间结束
