#include "hardware_test.h"
#include <cassert>
#include <cstdio>
int main() {
	hardware_test::setup();
	assert(pwm[ports::PWMA]==50 && pwm[ports::PWMB]==50);
	assert(levels[ports::AIN_1]==HIGH && levels[ports::BIN_2]==HIGH);
	nowUs+=250000; hardware_test::update();
	assert(Serial.output.str().find("tof1=")!=std::string::npos);
	assert(Serial.output.str().find("encA=")==std::string::npos);
	puts("Encoder-free hardware test checks passed");
}
