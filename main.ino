#include "base.h"

#include "ultrasonic.h"
#include "tof.h"
#include "servo.h"
#include "motor.h"

#include "control.h"
#include "hardware_test.h"
#include "runtime_debug.h"

#define ENABLE_WEB_DEBUG 1

#if ENABLE_WEB_DEBUG
#include "web_debug.h"
#endif

// true：传感器、电机联合测试，不执行其它逻辑
constexpr bool hardware_test_mode = false;
// true：启用 Wi-Fi 调试网页。关闭时串口调试仍可用。
constexpr bool enable_web_debug = ENABLE_WEB_DEBUG;
// 前进 PWM 在 calibration.h 的 DEFAULT_PWM_A/B 中配置。

static bool initializationFailed = false;

constexpr uint8_t MAX_INITIALIZATION_ATTEMPTS = 3;
constexpr uint32_t INITIALIZATION_RETRY_MS = 2000;
static uint8_t initializationAttempts = 0;
static uint32_t initializationFailedAt = 0;

void attemptInitialization() {
	initializationFailed = false;
	++initializationAttempts;

	motor::stopMotors();
	servo::disable();
	tof::setup_early();

	Serial.print(F("Initialization attempt "));
	Serial.println(initializationAttempts);

	ultrasonic::setup_late();

	if (!tof::setup_late()) {
		initializationFailed = true;
		control::recordTofInitFailure(tof::firstInitialized, tof::secondInitialized);

		Serial.print(F("ToF startup unavailable:"));
		runtime_debug::printSensorIssues(control::getTelemetry().sensorIssues);

		Serial.println();
	}

	servo::setup_late();
	motor::setup_late();

	if (!initializationFailed) {
		if constexpr (hardware_test_mode)
			hardware_test::setup();
		else if (!control::start()) {
			Serial.println("Control start failed: check calibration and configuration.");
			initializationFailed = true;
		}
	}

	if (initializationFailed) {
		motor::stopMotors();
		servo::disable();

		initializationFailedAt = static_cast<uint32_t>(millis());

		if (initializationAttempts < MAX_INITIALIZATION_ATTEMPTS)
			Serial.println(F("Initialization failed. Retrying in 2 seconds."));
		else
			Serial.println(F("Initialization failed 3 times. Vehicle stopped."));
	}
	else {
		Serial.println(F("Initialization succeeded."));
	}

#if ENABLE_WEB_DEBUG
	if constexpr (enable_web_debug)
		web_debug::publish(initializationFailed, hardware_test_mode);
#endif
}

void setup() {
	tof::setup_early();
	ultrasonic::setup_early();
	servo::setup_early();
	motor::setup_early();

	Serial.begin(115200);
	
#if ENABLE_WEB_DEBUG
	if constexpr (enable_web_debug)
		web_debug::begin();
#endif

	attemptInitialization();
}

void loop() {
	if (initializationFailed) {
		if (initializationAttempts < MAX_INITIALIZATION_ATTEMPTS &&
				static_cast<uint32_t>(millis()) - initializationFailedAt >= INITIALIZATION_RETRY_MS)
			attemptInitialization();
		return;
	}

	if constexpr (hardware_test_mode) {
		hardware_test::update();
		return;
	}

	control::update();

#if ENABLE_WEB_DEBUG
	if constexpr (enable_web_debug)
		web_debug::publish(false, false);
#endif

	runtime_debug::update();
	
	// 状态变化立即报告，详细调试数据每 1000 ms 输出一次。
	static control::Status previousStatus = control::Status::Stopped;
	const control::Status status = control::getTelemetry().status;
	static uint8_t previousIssues = 0;
	const uint8_t issues = control::getTelemetry().sensorIssues;

	if (status != previousStatus || issues != previousIssues) {
		Serial.print("Control status: ");
		Serial.print(static_cast<int>(status));
		Serial.print(F(" ("));
		runtime_debug::printStatus(status);
		Serial.print(F(") sensors="));
		runtime_debug::printSensorIssues(issues);
		Serial.println();
		previousIssues = issues;
		previousStatus = status;
	}
}
