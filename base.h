#pragma once

#include "calibration.h"

#include <ESP32Servo.h>
#include "src/vl53l0x_esp_idf/VL53L0X.h"

namespace ports {
constexpr int TOF_SDA = 8, TOF_SCL = 9;
constexpr int XSHUT_1 = 10, XSHUT_2 = 11;
constexpr int ULTRASONIC_TRIG = 12, ULTRASONIC_ECHO = 13;
constexpr int SERVO = 14;
constexpr int ENA = 4, IN_1 = 5, IN_2 = 6;
constexpr int ENB = 7, IN_3 = 15, IN_4 = 16;
}

namespace devices {
	VL53L0X tof1, tof2;
	Servo servo;
}
