#pragma once

#include "calibration.h"

#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32Servo.h>
#include "src/vl53l0x_esp_idf/VL53L0X.h"
#else
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#endif

namespace ports {
#if defined(CONFIG_IDF_TARGET_ESP32S3)
constexpr int TOF_SDA = 8, TOF_SCL = 9;
constexpr int XSHUT_1 = 10, XSHUT_2 = 11;
constexpr int ULTRASONIC_TRIG = 12, ULTRASONIC_ECHO = 13;
constexpr int SERVO = 14;
constexpr int ENA = 4, IN_1 = 5, IN_2 = 6;
constexpr int ENB = 7, IN_3 = 15, IN_4 = 16;
#else
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
#endif
}

namespace devices {
	VL53L0X tof1, tof2;
	Servo servo;
}
