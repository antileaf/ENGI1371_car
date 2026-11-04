#include "base.h"

#include "ultrasonic.h"
#include "tof.h"
#include "servo.h"
#include "motor.h"

#include "control.h"
#include "hardware_test.h"
#include "runtime_debug.h"

// true：传感器、电机联合测试，不执行其它逻辑
constexpr bool hardware_test_mode = false;
// 前进 PWM 在 calibration.h 的 DEFAULT_PWM_A/B 中配置。

static bool initializationFailed = false;

void setup() {
	ultrasonic::setup_early();
	servo::setup_early();
	motor::setup_early();

	Serial.begin(115200);

	ultrasonic::setup_late();
	if (!tof::setup_late())
		initializationFailed = true;
	servo::setup_late();
	motor::setup_late();

	if (!initializationFailed) {
		if (hardware_test_mode)
			hardware_test::setup();
		else if (!control::start()) {
			Serial.println("Control start failed: check calibration and configuration.");
			initializationFailed = true;
		}
	}

	if (initializationFailed) {
		motor::stopMotors();
		servo::disable();
		Serial.println(F("Initialization failed. Vehicle stopped."));
	}
}

void loop() {
	if (initializationFailed)
		return;

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
