#pragma once

#include "calibration.h"

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

namespace ports {
// 舵机
constexpr int SERVO = 12; // D12

// 超声波传感器
constexpr int ULTRASONIC_TRIG = A0; // output
constexpr int ULTRASONIC_ECHO = 13; // input

// ToF 传感器
constexpr int XSHUT_1 = 10; // output
constexpr int XSHUT_2 = 11; // output
// 两颗 ToF 共用 Wire 的硬件 I²C 总线，经典 Nano 上为 A4/A5。
constexpr int TOF_SDA = SDA, TOF_SCL = SCL;

// L298N 电机驱动板
constexpr int ENA = 5, ENB = 3; // A/B 通道使能
constexpr int IN_1 = 8, IN_2 = 7; // output
constexpr int IN_3 = 6, IN_4 = 4; // output
}

namespace devices {
	VL53L0X tof1, tof2;
	Servo servo;
}
