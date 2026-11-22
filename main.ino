#include "base.h"

#include "ultrasonic.h"
#include "tof.h"
#include "servo.h"
#include "motor.h"

#include "control.h"
#include "hardware_test.h"
#include "runtime_debug.h"
#include "boot_log.h"

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

constexpr uint32_t INITIALIZATION_WINDOW_US = 2000000UL;
constexpr uint32_t INITIALIZATION_PAUSE_US = 2000000UL;
static uint32_t initializationAttempts = 0;
static uint32_t initializationPhaseStartedUs = 0;
static uint32_t lastInitializationAttemptUs = 0;
static bool initializationPaused = false;

void attemptInitialization() {
	status_led::show(status_led::State::Initializing);
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
	status_led::update(!tof::firstInitialized, !tof::secondInitialized);

	if constexpr (hardware_test_mode) {
		hardware_test::setup();
		if (initializationFailed)
			Serial.println(F("Hardware test: continuing with available modules."));
	}
	else {
		if (!initializationFailed && !control::start()) {
			Serial.println("Control start failed: check calibration and configuration.");
			initializationFailed = true;
		}
	}

	if (initializationFailed && !hardware_test_mode) {
		motor::stopMotors();
		servo::disable();

		initializationPhaseStartedUs = static_cast<uint32_t>(micros());
		lastInitializationAttemptUs = initializationPhaseStartedUs;
		initializationPaused = false;
		Serial.println(F("Initialization failed. Retrying at control sampling interval for 2 seconds."));
	}
	else if (!initializationFailed) {
		Serial.println(F("Initialization succeeded."));
	}

#if ENABLE_WEB_DEBUG
	if constexpr (enable_web_debug)
		web_debug::publish(initializationFailed, hardware_test_mode);
#endif
}

// 仅恢复故障侧，不反复重置正常模块或拨动舵机。
void recoverInitialization() {
	++initializationAttempts;
	if (!tof::firstInitialized || !tof::secondInitialized)
		tof::retryFailedSensors();
	control::recordTofInitFailure(tof::firstInitialized, tof::secondInitialized);
	status_led::update(!tof::firstInitialized, !tof::secondInitialized);
	if (tof::firstInitialized && tof::secondInitialized && control::start()) {
		initializationFailed = false;
		Serial.println(F("Initialization recovered. Control warming up."));
	}
	else {
		motor::stopMotors();
		servo::disable();

	}

}

// 2 秒尝试窗口与 2 秒休息交替；初始化本身耗时超过周期时不追赶补执行。
void updateInitializationRecovery() {
	const uint32_t now = static_cast<uint32_t>(micros());
	if (initializationPaused) {
		if (now - initializationPhaseStartedUs < INITIALIZATION_PAUSE_US) return;
		initializationPaused = false;
		initializationPhaseStartedUs = now;
		lastInitializationAttemptUs = now - control::CONTROL_PERIOD_US;
		Serial.println(F("Initialization: starting next 2-second retry window."));
	}
	if (now - initializationPhaseStartedUs >= INITIALIZATION_WINDOW_US) {
		initializationPaused = true;
		initializationPhaseStartedUs = now;
		Serial.println(F("Initialization: pausing retries for 2 seconds."));
		return;
	}
	if (now - lastInitializationAttemptUs < control::CONTROL_PERIOD_US) return;
	lastInitializationAttemptUs = now;
	recoverInitialization();
	const uint32_t finished = static_cast<uint32_t>(micros());
	if (initializationFailed && finished - initializationPhaseStartedUs >= INITIALIZATION_WINDOW_US) {
		initializationPaused = true;
		initializationPhaseStartedUs = finished;
		Serial.println(F("Initialization: pausing retries for 2 seconds."));
	}
}

void setup() {
	tof::setup_early();
	ultrasonic::setup_early();
	servo::setup_early();
	motor::setup_early();
	Serial.begin(115200);
	// 先保存启动记录，避免在白灯等待期间再次复位而遗漏本次启动。
	boot_log::begin();
	// 每次开机仅闪一次白灯；先配置硬件引脚，模块重试不执行此段。
	status_led::show(status_led::State::Boot);
	delay(500);
	status_led::show(status_led::State::Initializing);

	serial_commands::printBootLog = boot_log::printRecords;
	Serial.println(F("Send boot followed by Enter to print boot history."));
	
#if ENABLE_WEB_DEBUG
	if constexpr (enable_web_debug)
		web_debug::begin();
#endif

	attemptInitialization();
}

void loop() {
	if constexpr (hardware_test_mode) {
		hardware_test::update();
		return;
	}
	serial_commands::update(false);
	if (initializationFailed) {
		updateInitializationRecovery();
		runtime_debug::update();
#if ENABLE_WEB_DEBUG
		if constexpr (enable_web_debug) web_debug::publish(initializationFailed, false);
#endif
		return;
	}

	control::update();
	const uint8_t tofIssues = control::getTelemetry().sensorIssues;
	const control::Status ledStatus = control::getTelemetry().status;
	const bool obstacleStopped = ledStatus == control::Status::Obstacle ||
		ledStatus == control::Status::WaitingToRestart || ledStatus == control::Status::ObstacleLocked;
	status_led::update(tofIssues & control::ToF1Invalid, tofIssues & control::ToF2Invalid, obstacleStopped);

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
