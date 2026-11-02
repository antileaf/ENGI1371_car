#pragma once

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

namespace ports {
// 舵机
constexpr int SERVO = 9;

// 超声波传感器
constexpr int ULTRASONIC_TRIG = 10; // output
constexpr int ULTRASONIC_ECHO = 11; // input

// ToF 传感器
constexpr int XSHUT_1 = A2; // output
constexpr int SDA_1 = SDA, SCL_1 = SCL;
constexpr int XSHUT_2 = A3; // output
constexpr int SDA_2 = A4, SCL_2 = A5;

// 驱动板
constexpr int PWMA = 5, PWMB = 6; // analog output
constexpr int AIN_1 = 4, AIN_2 = 7; // output
constexpr int BIN_1 = 8, BIN_2 = 12; // output
constexpr int STBY = 13; // output
}

namespace devices {
	VL53L0X tof1, tof2;
	Servo servo;
}
