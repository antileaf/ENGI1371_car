#include "sketch.h"
#include <cassert>
#include <cstdio>
int main() {
	mockTofInitFails = true;
	setup();
	assert(initializationFailed && initializationAttempts == 1);
	// 采样周期之前不重试，到期后重试；2 秒窗口后进入休息。
	nowUs = lastInitializationAttemptUs + control::CONTROL_PERIOD_US - 1;
	loop(); assert(initializationAttempts == 1);
	++nowUs; loop(); assert(initializationAttempts == 2);
	for (unsigned round = 0; round < 3; ++round) {
		while (!initializationPaused) { nowUs += control::CONTROL_PERIOD_US; loop(); }
		const uint32_t attempts = initializationAttempts;
		assert(pwm[ports::ENA] == 0 && pwm[ports::ENB] == 0 && !devices::servo.enabled);
		nowUs = initializationPhaseStartedUs + INITIALIZATION_PAUSE_US - 1;
		loop(); assert(initializationAttempts == attempts);
		++nowUs; loop();
		assert(!initializationPaused && initializationAttempts == attempts + 1);
	}
	// 第一侧已恢复时不应被下一轮重初始化覆盖。
	tof::firstInitialized = true;
	devices::tof1.mm = 777;
	mockTofInitFails = false;
	nowUs = lastInitializationAttemptUs + control::CONTROL_PERIOD_US;
	loop();
	assert(!initializationFailed && devices::tof1.mm == 777);
	assert(control::getTelemetry().status == control::Status::WarmingUp);
	assert(pwm[ports::ENA] == 0 && pwm[ports::ENB] == 0);
	for (unsigned i = 0; i < 50; ++i) { nowUs += control::CONTROL_PERIOD_US; loop(); }
	assert(control::getTelemetry().status == control::Status::Running);
	puts("Initialization retry, persistent recovery and automatic restart checks passed");
}
