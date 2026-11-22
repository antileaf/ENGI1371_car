#include "hardware_test.h"
#include <cassert>
#include <cstdio>
int main() {
	tof::firstInitialized=tof::secondInitialized=true;
	servo::setup_late();
	servo::disable();
	servo::enable(73);
	assert(devices::servo.attached() && devices::servo.angle==73);
	devices::tof1.last_status=1;
	float left, right;
	assert(!tof::readDistances(left, right) && isnan(left) && isfinite(right));
	devices::tof1.last_status=0;
	hardware_test::setup();
	assert(devices::servo.angle==hardware_test::SERVO_ANGLE);
	Serial.input="12"; hardware_test::update();
	assert(devices::servo.angle==hardware_test::SERVO_ANGLE);
	Serial.input="0\r\n"; hardware_test::update();
	assert(devices::servo.angle==120);
	Serial.input="181\n-1\n9x0\n"; hardware_test::update();
	assert(devices::servo.angle==120);
	Serial.input="0\n"; hardware_test::update(); assert(devices::servo.angle==0);
	Serial.input="180\n"; hardware_test::update(); assert(devices::servo.angle==180);
	// boot 支持分帧和 CRLF，不改变舵机；正式运行也能查询但不能手动转向。
	static int bootRequests = 0;
	serial_commands::printBootLog = []() { ++bootRequests; };
	Serial.input="bo"; hardware_test::update(); assert(bootRequests==0);
	Serial.input="ot\r\n"; hardware_test::update();
	assert(bootRequests==1 && devices::servo.angle==180);
	Serial.input="boot\n90\n"; serial_commands::update(false);
	assert(bootRequests==2 && devices::servo.angle==180);
	Serial.input="12345678901234567890boot\nboot\n"; hardware_test::update();
	assert(bootRequests==3 && devices::servo.angle==180);
	assert(pwm[ports::ENA]==hardware_test::TEST_PWM && pwm[ports::ENB]==hardware_test::TEST_PWM);
	assert(levels[ports::IN_1]==HIGH && levels[ports::IN_4]==HIGH);
	nowUs+=250000; hardware_test::update();
	assert(Serial.output.str().find("tof1=")!=std::string::npos);
	assert(Serial.output.str().find("encA=")==std::string::npos);
	motor::brakeMotors();
	assert(pwm[ports::ENA]==255 && pwm[ports::ENB]==255);
	assert(levels[ports::IN_1]==LOW && levels[ports::IN_2]==LOW);
	assert(levels[ports::IN_3]==LOW && levels[ports::IN_4]==LOW);
	motor::stopMotors();
	assert(pwm[ports::ENA]==0 && pwm[ports::ENB]==0);
	// 短暂失败可自行恢复；连续失败只重启故障侧，并保留正常侧对象。
	tof::firstInitialized = tof::secondInitialized = true;
	tof::firstFailures = tof::secondFailures = 0;
	devices::tof1.mm = 456;
	devices::tof2.timeout = true;
	for (unsigned i = 0; i < tof::READ_FAILURE_LIMIT - 1; ++i)
		assert(!tof::readDistance(devices::tof2, right));
	assert(tof::secondInitialized);
	devices::tof2.timeout = false;
	assert(tof::readDistance(devices::tof2, right) && tof::secondFailures == 0);
	devices::tof2.timeout = true;
	for (unsigned i = 0; i < tof::READ_FAILURE_LIMIT; ++i)
		assert(!tof::readDistance(devices::tof2, right));
	assert(!tof::secondInitialized && tof::firstInitialized);
	assert(levels[ports::XSHUT_2] == LOW && !tof::recoveryDue());
	assert(!tof::readDistances(left, right) && isnan(right) && left > .45f);
	nowUs += (tof::RECOVERY_PERIOD_MS - 1) * 1000;
	assert(!tof::recoveryDue());
	nowUs += 1000;
	assert(tof::recoveryDue());
	tof::retryFailedSensors();
	assert(tof::secondInitialized && tof::secondFailures == 0);
	assert(devices::tof1.mm == 456 && !tof::recoveryDue());
	assert(tof::readDistances(left, right));
	puts("Encoder-free hardware test and single-side ToF recovery checks passed");
}
