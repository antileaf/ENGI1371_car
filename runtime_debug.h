#pragma once

#include "control.h"

namespace runtime_debug {

constexpr uint32_t OUTPUT_PERIOD_MS = 1000;

inline void printStatus(control::Status status) {
	switch (status) {
		case control::Status::Stopped: Serial.print(F("Stopped")); break;
		case control::Status::WarmingUp: Serial.print(F("WarmingUp")); break;
		case control::Status::Running: Serial.print(F("Running")); break;
		case control::Status::CalibrationMissing: Serial.print(F("CalibrationMissing")); break;
		case control::Status::SensorFault: Serial.print(F("SensorFault")); break;
		case control::Status::Obstacle: Serial.print(F("Obstacle")); break;
		case control::Status::TimingFault: Serial.print(F("TimingFault")); break;
		case control::Status::InvalidTarget: Serial.print(F("InvalidTarget")); break;
		case control::Status::WaitingToRestart: Serial.print(F("WaitingToRestart")); break;
		case control::Status::ObstacleLocked: Serial.print(F("ObstacleLocked")); break;
	}
}

inline void printSensorIssues(uint8_t issues) {
	if (!issues) Serial.print(F("OK"));
	if (issues & control::ToF1Invalid) Serial.print(F(" ToF1"));
	if (issues & control::ToF2Invalid) Serial.print(F(" ToF2"));
	if (issues & control::FrontNoEcho) Serial.print(F(" Ultrasonic:NO_ECHO"));
	if (issues & control::FrontInvalid) Serial.print(F(" Ultrasonic:INVALID_ECHO"));
}

// 只输出控制器缓存，不重新测距，避免干扰控制周期。
// 停车/故障期间部分值会保留上次读数；尚未采样的字段可能为初始零值。
// 不使用 delay；循环有阻塞时在下一次调用补报一次，不连续补发。
inline void update() {
	static uint32_t lastOutputMs = 0;
	const uint32_t now = static_cast<uint32_t>(millis());
	if (now - lastOutputMs < OUTPUT_PERIOD_MS) return;
	lastOutputMs = now;
	const control::Telemetry &data = control::getTelemetry();
	Serial.print(F("[control cached] t_ms=")); Serial.print(now);
	Serial.print(F(" status=")); printStatus(data.status);
	Serial.print(F(" sensors=")); printSensorIssues(data.sensorIssues);
	Serial.print(F(" front_m="));
	if (isnan(data.frontDistance)) Serial.print(F("NO_ECHO"));
	else Serial.print(data.frontDistance, 3);
	Serial.print(F(" tof_left_m="));
	if (isfinite(data.tofLeft)) Serial.print(data.tofLeft, 3);
	else Serial.print(F("NA"));
	Serial.print(F(" tof_right_m="));
	if (isfinite(data.tofRight)) Serial.print(data.tofRight, 3);
	else Serial.print(F("NA"));
	Serial.print(F(" offset_raw_ratio=")); Serial.print(data.rawOffset, 3);
	Serial.print(F(" offset_ratio=")); Serial.print(data.offset, 3);
	Serial.print(F(" offset_rate_per_s=")); Serial.print(data.offsetRate, 3);
	Serial.print(F(" p_rad=")); Serial.print(data.pTerm, 3);
	Serial.print(F(" d_rad=")); Serial.print(data.dTerm, 3);
	Serial.print(F(" steer_rad=")); Serial.print(data.appliedPhi, 3);
	Serial.print(F(" servo_deg=")); Serial.print(data.servoDegrees, 1);
	Serial.print(F(" pwmA=")); Serial.print(data.pwmA);
	Serial.print(F(" pwmB=")); Serial.print(data.pwmB);
	Serial.println();
}

} // namespace runtime_debug
