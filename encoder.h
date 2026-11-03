#pragma once

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include "base.h"

#if defined(__AVR__)
#include <util/atomic.h>
#endif

namespace encoder {

using calibration::WheelCalibration;
using calibration::GEAR_RATIO;
using calibration::MOTOR_SHAFT_PULSES_PER_REV;
using calibration::COUNTS_PER_WHEEL_REV;
using calibration::motorACalibration;
using calibration::motorBCalibration;

// 最小测速间隔为 10 ms；实际速度使用两次采样之间的真实时间计算。
constexpr uint32_t MIN_SAMPLE_US = 10000UL;

namespace detail {
// 无符号累加允许正反转跨零和计数溢出，不触发有符号整数溢出。
static volatile uint32_t motorATicks = 0;
static volatile uint32_t motorBTicks = 0;
static bool initialized = false;
static uint32_t previousMotorA = 0;
static uint32_t previousMotorB = 0;
static uint32_t previousTimeUs = 0;

inline void onMotorAPulse() {
	// B 为低时记正；安装方向差异由标定参数 direction 修正。
	if (digitalRead(ports::ENCODER_B1) == LOW)
		motorATicks++;
	else
		motorATicks--;
}

inline void onMotorBPulse() {
	if (digitalRead(ports::ENCODER_B2) == LOW)
		motorBTicks++;
	else
		motorBTicks--;
}

inline bool validCalibration(const WheelCalibration &calibration) {
	return isfinite(calibration.diameterMetres) &&
			calibration.diameterMetres > 0.0f &&
			isfinite(calibration.countsPerWheelRev) && 
			calibration.countsPerWheelRev > 0.0f &&
			(calibration.direction == 1 || calibration.direction == -1);
}
} // namespace detail

struct Counts {
	uint32_t motorA;    // 原始累计计数，尚未应用方向修正，允许无符号回绕
	uint32_t motorB;
	uint32_t timeUs;  // 与计数一起采样的 micros() 时间戳
};

// 仅从主循环调用，不要在中断服务函数中调用。
// AVR 保存并恢复原中断状态；其他平台要求调用时中断已开启。
// 此实现适用于中断与主循环在同一核心执行的 Arduino 控制板。
inline Counts readCounts() {
	Counts counts;

#if defined(__AVR__)
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		counts.motorA = detail::motorATicks;
		counts.motorB = detail::motorBTicks;
		counts.timeUs = static_cast<uint32_t>(micros());
	}
#else
	noInterrupts();
	counts.motorA = detail::motorATicks;
	counts.motorB = detail::motorBTicks;
	counts.timeUs = static_cast<uint32_t>(micros());
	interrupts();
#endif

	return counts;
}

// 求两次原始计数间的有符号变化量，兼容正反转及无符号回绕。
// 两次采样间的计数变化绝对值必须小于 2^31。
inline int32_t countDelta(uint32_t current, uint32_t previous) {
	const uint32_t difference = current - previous;
	if (difference <= INT32_MAX) return static_cast<int32_t>(difference);
	return -static_cast<int32_t>(UINT32_MAX - difference) - 1;
}

// 重新开始测速窗口，不清除累计计数，适合停车后重新开始采样。
inline void resetSpeedWindow() {
	const Counts counts = readCounts();
	detail::previousMotorA = counts.motorA;
	detail::previousMotorB = counts.motorB;
	detail::previousTimeUs = counts.timeUs;
}

// 初始化 A/B 输入并注册 A 相上升沿中断；不控制电机。
// 返回 true 只代表中断已配置，不代表轮速标定已完成。
inline bool setup() {
	if (detail::initialized) return true;
	const int motorAInterrupt = digitalPinToInterrupt(ports::ENCODER_A1);
	const int motorBInterrupt = digitalPinToInterrupt(ports::ENCODER_A2);
	if (motorAInterrupt == NOT_AN_INTERRUPT || motorBInterrupt == NOT_AN_INTERRUPT
		|| motorAInterrupt == motorBInterrupt) return false;

	pinMode(ports::ENCODER_A1, INPUT_PULLUP);
	pinMode(ports::ENCODER_B1, INPUT_PULLUP);
	pinMode(ports::ENCODER_A2, INPUT_PULLUP);
	pinMode(ports::ENCODER_B2, INPUT_PULLUP);
	attachInterrupt(motorAInterrupt, detail::onMotorAPulse, RISING);
	attachInterrupt(motorBInterrupt, detail::onMotorBPulse, RISING);
	resetSpeedWindow();
	detail::initialized = true;
	return true;
}

inline bool calibrationReady() {
	return detail::validCalibration(motorACalibration) &&
			detail::validCalibration(motorBCalibration);
}

// 返回电机 A、B 所驱动车轮的平均线速度，单位：米/秒，车身前进为正。
// 未初始化、未标定或采样时间不足时返回 false，两个输出均为 NAN。
// 调用方只在返回 true 时写入历史样本；不应把失败输出当作零速度。
// 两次有效采样间隔须小于 micros() 的一次完整回绕周期（约 71 分钟）。
inline bool readSpeeds(float &vMotorA, float &vMotorB) {
	vMotorA = NAN;
	vMotorB = NAN;
	if (!detail::initialized || !calibrationReady())
		return false;

	const Counts counts = readCounts();
	const uint32_t elapsedUs = counts.timeUs - detail::previousTimeUs;

	if (elapsedUs < MIN_SAMPLE_US)
		return false;

	const int32_t deltaMotorA = countDelta(counts.motorA, detail::previousMotorA);
	const int32_t deltaMotorB = countDelta(counts.motorB, detail::previousMotorB);
	
	detail::previousMotorA = counts.motorA;
	detail::previousMotorB = counts.motorB;
	detail::previousTimeUs = counts.timeUs;

	const float seconds = elapsedUs * 1.0e-6f;
	const float motorASpeed = static_cast<float>(deltaMotorA) * motorACalibration.direction
		* (PI * motorACalibration.diameterMetres / motorACalibration.countsPerWheelRev)
		/ seconds;
	const float motorBSpeed = static_cast<float>(deltaMotorB) * motorBCalibration.direction
		* (PI * motorBCalibration.diameterMetres / motorBCalibration.countsPerWheelRev)
		/ seconds;
	
	if (!isfinite(motorASpeed) || !isfinite(motorBSpeed))
		return false;
	
	vMotorA = motorASpeed;
	vMotorB = motorBSpeed;

	return true;
}

} // namespace encoder
