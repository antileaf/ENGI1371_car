#pragma once

#include <Arduino.h>

// 实车标定及安装参数；控制增益、采样周期和避障策略仍在各自模块。
namespace calibration {

// 后轮轮距 W、轴距 L
constexpr float W = 0.2f, L = 0.3f; // TODO: to be determined

// 传感器与电机安装方向
constexpr bool TOF1_IS_LEFT = false;
constexpr bool MOTOR_A_IS_LEFT = false;
constexpr int LEFT_MOTOR_DIRECTION = +1, RIGHT_MOTOR_DIRECTION = -1;
constexpr float MAX_STEER = 0.34f; // 弧度，约 19.5 度，留在当前 ±20 度标定范围内

struct SteeringCalibrationPoint {
	float wheelDeg; // 等效前轮转角，左转为正，单位：度
	float servoDeg; // 传给 Servo::write() 的舵机指令角度，单位：度
};

// 标定时统一使用自行车模型的等效前轮转角，
// 并非阿克曼转向机构的内外侧前轮实际转角。
constexpr SteeringCalibrationPoint steeringCalibration[] = {
	{-20.0f, 120.0}, // 右转 20 度
	{-10.0f, 100.0}, // 右转 10 度
	{  0.0f, 75.0}, // 前轮归中
	{ 10.0f, 50.0}, // 左转 10 度
	{ 20.0f, 40.0}  // 左转 20 度
};

constexpr int STEERING_POINT_COUNT = sizeof(steeringCalibration) / sizeof(steeringCalibration[0]);

constexpr float SERVO_CENTRE_DEG = steeringCalibration[STEERING_POINT_COUNT / 2].servoDeg;

// 编码器 1 对应电机 A，编码器 2 对应电机 B；接口始终按 A、B 顺序输出。
// 使用 A 相上升沿中断、读取 B 相判向，即单倍频计数。
// GM37-520 有不同编码器及减速比版本，不能仅凭型号确定每圈计数。
// 若资料给出电机轴 A 相每圈 P 个脉冲、减速比 G，且车轮直连输出轴，
// 则本接口的每轮计数为 P * G，不乘 2 或 4。
// 也可以让车轮沿同一方向转若干整圈，用原始计数变化除以圈数标定。
struct WheelCalibration {
	float diameterMetres;     // 车轮有效直径，单位：米
	float countsPerWheelRev;  // 车轮转一圈的 A 相上升沿数量
	int direction;           // 方向修正：只能填 +1 或 -1，使前进轮速为正
};

// 厂家参数：https://www.xytmotors.com/encoder/151.html
// 12 V、空载 178 rpm 对应 JGB37-520-12100，标称减速比为 56:1。
// 页面参数表及尺寸图未注明电机轴 A 相每圈脉冲数，暂定 11。
constexpr float GEAR_RATIO = 56.0f;
const float MOTOR_SHAFT_PULSES_PER_REV = 11; // TODO: to be determined
const float COUNTS_PER_WHEEL_REV = MOTOR_SHAFT_PULSES_PER_REV * GEAR_RATIO;

// 车轮直径已确认为 65 mm
// 确认脉冲数后更新上面的暂定值，或分别将 A/B 的每轮计数替换为实测值。
// 电机降压至 5 V 不改变每圈计数，也无需对测速结果乘电压比例。
// 未标定时仍可读取原始计数；方向修正需按实车前进方向检查。
// 电机 A（编码器 1）
const WheelCalibration motorACalibration  = {0.065f, COUNTS_PER_WHEEL_REV, LEFT_MOTOR_DIRECTION};
// 电机 B（编码器 2）
const WheelCalibration motorBCalibration = {0.065f, COUNTS_PER_WHEEL_REV, RIGHT_MOTOR_DIRECTION};

} // namespace calibration
