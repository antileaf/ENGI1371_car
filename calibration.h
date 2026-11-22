#pragma once
#include <stdint.h>

namespace calibration {

// 靠边退让：无后方测距，倒车时间和 PWM 需按实车后方空间调校。
constexpr bool EDGE_RECOVERY_ENABLED = true;
constexpr float EDGE_NEAR_M = 0.25f, EDGE_DISTANCE_RATIO = 2.0f;
constexpr uint32_t EDGE_CONFIRM_US = 0.75 * 1000000UL;
constexpr uint32_t EDGE_STOP_US = 200000UL, EDGE_SERVO_WAIT_US = 500000UL;
constexpr uint32_t EDGE_REVERSE_US = 1000000UL, EDGE_COOLDOWN_US = 3000000UL;
constexpr int EDGE_REVERSE_PWM = 145;

struct SteeringCalibrationPoint {
	float wheelDeg; // 等效前轮转角，左转为正，单位：度
	float servoDeg; // 传给 Servo::write() 的舵机指令角度，单位：度
};

// 标定时统一使用自行车模型的等效前轮转角，
// 并非阿克曼转向机构的内外侧前轮实际转角。
constexpr SteeringCalibrationPoint steeringCalibration[] = {
	{-20.0f, 120.0}, // 右转 20 度
	{-10.0f, 104.0}, // 右转 10 度
	{  0.0f, 85.0}, // 前轮归中
	{ 10.0f, 55.0}, // 左转 10 度
	{ 20.0f, 45.0}  // 左转 20 度
};

constexpr int STEERING_POINT_COUNT = sizeof(steeringCalibration) / sizeof(steeringCalibration[0]);

constexpr int CENTRE_INDEX = STEERING_POINT_COUNT / 2;
static_assert(steeringCalibration[CENTRE_INDEX].wheelDeg == 0, "Middle calibration point must be straight ahead");
constexpr float SERVO_CENTRE_DEG = steeringCalibration[CENTRE_INDEX].servoDeg;
constexpr float MAX_STEER_DEG = 20.0f; // 最大等效前轮转角，单位：度

// 低速前进控制参数：左右位置和电机正方向需按实车确认。
constexpr bool TOF1_IS_LEFT = true;
constexpr float TOF_MAX_DISTANCE_M = 3.0f; // ToF 控制计算距离上限，单位：米；超过上限按此值计算。
// 前进时各电机的输出符号；A 在右侧，B 在左侧。
constexpr int MOTOR_A_DIRECTION = +1, MOTOR_B_DIRECTION = -1;
static_assert((MOTOR_A_DIRECTION == 1 || MOTOR_A_DIRECTION == -1) &&
	(MOTOR_B_DIRECTION == 1 || MOTOR_B_DIRECTION == -1), "Motor direction must be +1 or -1");

// 两侧独立调平的前进 PWM 绝对值。
constexpr int MIN_DRIVE_PWM = 70; // 稳态行驶目标及转弯内轮下限，起步基础值单独配置。
constexpr int START_PWM = 60; // 起步加速的基础 PWM。
constexpr int TOF_DEGRADED_PWM = 90; // 行驶中 ToF 数据不可用或历史不足时，两轮降速 PWM。
constexpr int DEFAULT_PWM_A = 145, DEFAULT_PWM_B = 145;
// 满舵时内侧后轮的 PWM 最大降幅
constexpr float MAX_INNER_PWM_REDUCTION = 0.5f;
static_assert(MAX_INNER_PWM_REDUCTION >= 0 && MAX_INNER_PWM_REDUCTION <= 1,
	"Inner wheel PWM reduction must be between 0 and 1");

} // namespace calibration
