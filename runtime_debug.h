#pragma once

#include "control.h"

namespace runtime_debug {

constexpr uint32_t OUTPUT_PERIOD_MS = 1000;

inline void printStatus(control::Status status) {
	switch (status) {
		case control::Status::Stopped: Serial.print(F("Stopped")); break;
		case control::Status::WarmingUp: Serial.print(F("WarmingUp")); break;
		case control::Status::LowSpeed: Serial.print(F("LowSpeed")); break;
		case control::Status::Running: Serial.print(F("Running")); break;
		case control::Status::CalibrationMissing: Serial.print(F("CalibrationMissing")); break;
		case control::Status::SensorFault: Serial.print(F("SensorFault")); break;
		case control::Status::Obstacle: Serial.print(F("Obstacle")); break;
		case control::Status::TimingFault: Serial.print(F("TimingFault")); break;
		case control::Status::EstimationFault: Serial.print(F("EstimationFault")); break;
		case control::Status::DriveFault: Serial.print(F("DriveFault")); break;
		case control::Status::InvalidTarget: Serial.print(F("InvalidTarget")); break;
		case control::Status::WaitingToRestart: Serial.print(F("WaitingToRestart")); break;
		case control::Status::ObstacleLocked: Serial.print(F("ObstacleLocked")); break;
	}
}

// 只输出控制器缓存，不重新测距或读取编码器，避免干扰测速窗口。
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
	Serial.print(F(" front_m="));
	if (isnan(data.frontDistance)) Serial.print(F("NO_ECHO"));
	else Serial.print(data.frontDistance, 3);
	Serial.print(F(" tof_left_m="));
	if (isfinite(data.tofLeft)) Serial.print(data.tofLeft, 3);
	else Serial.print(F("NA"));
	Serial.print(F(" tof_right_m="));
	if (isfinite(data.tofRight)) Serial.print(data.tofRight, 3);
	else Serial.print(F("NA"));
	Serial.print(F(" vL_mps=")); Serial.print(data.measuredLeft, 3);
	Serial.print(F(" vR_mps=")); Serial.print(data.measuredRight, 3);
	Serial.print(F(" encL_mps=")); Serial.print(data.encoderLeft, 3);
	Serial.print(F(" encR_mps=")); Serial.print(data.encoderRight, 3);
	Serial.print(F(" sourceL="));
	Serial.print(data.fallbackLeft ? (data.fallbackLeftMeasured ? F("PWM_LEARNED") : F("PWM_COMMAND")) : F("ENCODER"));
	Serial.print(F(" sourceR="));
	Serial.print(data.fallbackRight ? (data.fallbackRightMeasured ? F("PWM_LEARNED") : F("PWM_COMMAND")) : F("ENCODER"));
	Serial.print(F(" targetL_mps=")); Serial.print(data.targetLeft, 3);
	Serial.print(F(" targetR_mps=")); Serial.print(data.targetRight, 3);
	Serial.print(F(" x_m=")); Serial.print(data.car.x, 3);
	Serial.print(F(" theta_rad=")); Serial.print(data.car.theta, 3);
	Serial.print(F(" kappa_1pm=")); Serial.print(data.car.kappa, 3);
	Serial.print(F(" steer_rad=")); Serial.print(data.appliedPhi, 3);
	Serial.print(F(" servo_deg=")); Serial.print(data.servoDegrees, 1);
	Serial.print(F(" pwmL=")); Serial.print(data.pwmLeft);
	Serial.print(F(" pwmR=")); Serial.println(data.pwmRight);
}

} // namespace runtime_debug
