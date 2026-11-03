#pragma once

#include "calibration.h"

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

constexpr int DATA_PER_SEC = 20;
constexpr int DATA_SEP = 1000 / DATA_PER_SEC;
constexpr int K = 10;

// 保留原名称，尺寸统一在 calibration.h 中修改。
using calibration::W;
using calibration::L;

float dl[K], dr[K], vl[K], vr[K];

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

// 编码器
constexpr int ENCODER_A1 = 2, ENCODER_B1 = A0;
constexpr int ENCODER_A2 = 3, ENCODER_B2 = A1;

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
