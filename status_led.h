#pragma once

#include "base.h"

namespace status_led {

constexpr uint8_t BRIGHTNESS = 16; // 降低亮度，避免近距离调试刺眼。
constexpr uint32_t OBSTACLE_BLINK_PERIOD_MS = 2000;
constexpr uint32_t OBSTACLE_OFF_MS = 500;
enum class State : uint8_t { Boot, Initializing, Healthy, LeftFault, RightFault, BothFaults, Off };

inline void show(State state) {
	static int previous = -1;
	if (previous == static_cast<int>(state)) return;
	previous = static_cast<int>(state);
	uint8_t red = 0, green = 0, blue = 0;
	switch (state) {
		case State::Off: break;
		case State::Boot: red = green = blue = BRIGHTNESS; break;
		case State::Initializing: blue = BRIGHTNESS; break;
		case State::Healthy: green = BRIGHTNESS; break;
		case State::LeftFault: red = green = BRIGHTNESS; break;
		case State::RightFault: red = blue = BRIGHTNESS; break;
		case State::BothFaults: red = BRIGHTNESS; break;
	}
	// 仅状态变化时发送颜色，不增加延时或额外读取传感器。
	rgbLedWrite(ports::STATUS_RGB, red, green, blue);
}

// 输入按传感器编号排列，灯色按标定配置映射到实际左右侧。
inline void update(bool firstFault, bool secondFault, bool obstacleStopped = false) {
	static bool wasObstacleStopped = false;
	static uint32_t blinkStartedMs = 0;
	const uint32_t now = static_cast<uint32_t>(millis());
	if (obstacleStopped && !wasObstacleStopped) blinkStartedMs = now;
	wasObstacleStopped = obstacleStopped;
	// 障碍停车期间每周期亮 1.5 秒、灭 0.5 秒；恢复行驶立即恢复常亮。
	if (obstacleStopped && (now - blinkStartedMs) % OBSTACLE_BLINK_PERIOD_MS >=
			OBSTACLE_BLINK_PERIOD_MS - OBSTACLE_OFF_MS) {
		show(State::Off);
		return;
	}
	const bool leftFault = calibration::TOF1_IS_LEFT ? firstFault : secondFault;
	const bool rightFault = calibration::TOF1_IS_LEFT ? secondFault : firstFault;
	show(leftFault ? (rightFault ? State::BothFaults : State::LeftFault)
		: (rightFault ? State::RightFault : State::Healthy));
}

} // namespace status_led
