#include "hardware_test.h"
#include <cassert>
#include <cstdio>
int main() {
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
	assert(pwm[ports::ENA]==50 && pwm[ports::ENB]==50);
	assert(levels[ports::IN_1]==HIGH && levels[ports::IN_4]==HIGH);
	nowUs+=250000; hardware_test::update();
	assert(Serial.output.str().find("tof1=")!=std::string::npos);
	assert(Serial.output.str().find("encA=")==std::string::npos);
	motor::brakeMotors();
	assert(levels[ports::ENA]==HIGH && levels[ports::ENB]==HIGH);
	assert(levels[ports::IN_1]==LOW && levels[ports::IN_2]==LOW);
	assert(levels[ports::IN_3]==LOW && levels[ports::IN_4]==LOW);
	motor::stopMotors();
	assert(levels[ports::ENA]==LOW && levels[ports::ENB]==LOW);
	puts("Encoder-free hardware test checks passed");
}
