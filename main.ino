#include "base.h"

#include "ultrasonic.h"
#include "tof.h"
#include "servo.h"
#include "motor.h"
#include "encoder.h"

#include "state.h"
#include "pure_pursuit.h"
#include "control.h"
#include "hardware_test.h"
#include "runtime_debug.h"

// true：传感器、电机、编码器联合测试，不执行其它逻辑
constexpr bool hardware_test_mode = false;
constexpr float target_speed = 0.24f; // 目标速度

void setup() {
	ultrasonic::setup_early();
	servo::setup_early();
	motor::setup_early();

	Serial.begin(115200);

	if (!encoder::setup()) {
		Serial.println("Encoder interrupt setup failed!");
		motor::stopMotors();
		while (true) {}
	}

	ultrasonic::setup_late();
	tof::setup_late();
	servo::setup_late();
	motor::setup_late();
	encoder::resetSpeedWindow();

	if (hardware_test_mode)
		hardware_test::setup();
	else if (!control::start(target_speed))
		Serial.println("Control start failed: check calibration and configuration.");
}

void loop() {
	if (hardware_test_mode) {
		hardware_test::update();
		return;
	}

	control::update();
	runtime_debug::update();
	
	// 状态变化立即报告，详细调试数据每 1000 ms 输出一次。
	static control::Status previousStatus = control::Status::Stopped;
	const control::Status status = control::getTelemetry().status;
	if (status != previousStatus) {
		Serial.print("Control status: ");
		Serial.print(static_cast<int>(status));
		Serial.print(F(" ("));
		runtime_debug::printStatus(status);
		Serial.println(F(")"));
		previousStatus = status;
	}
}

// v = (v_l + v_r) / 2
// 车辆偏移 x = (d_r - d_l) / 2 cosθ ≈ (d_r - d_l) / 2
// 车辆角速度 alpha = (v_r - v_l) / W
 
// 车辆偏角 θ = arcsin(x' / v) ≈ x' / v, θ_k ≈ (x_k - x_{k - 1}) / v_k Δt
