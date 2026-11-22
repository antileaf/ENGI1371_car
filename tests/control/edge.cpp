#include "control.h"
#include <cassert>
#include <cstdio>
using namespace control;
void tick() { nowUs += CONTROL_PERIOD_US; update(); }
void distances(int left, int right) {
	devices::tof1.mm = TOF1_IS_LEFT ? left : right;
	devices::tof2.mm = TOF1_IS_LEFT ? right : left;
}
void fresh() {
	nowUs = 0; echoUs = 5800;
	tof::firstInitialized = tof::secondInitialized = true;
	tof::firstFailures = tof::secondFailures = 0;
	devices::tof1 = VL53L0X(); devices::tof2 = VL53L0X();
	runtime::edgeCooldown = false;
	assert(start(100, 110));
	for (int i = 0; i < 40; ++i) tick();
}
void until(Status expected, int limit = 80) {
	for (int i = 0; i < limit && getTelemetry().status != expected; ++i) tick();
	assert(getTelemetry().status == expected);
}
void trigger(bool left = true) {
	distances(left ? 100 : 400, left ? 400 : 100);
	tick();
	// 根据配置检查触发前的状态，包含每轮超声波触发脉冲耗时。
	while (nowUs - runtime::edgeSince + CONTROL_PERIOD_US + 12 < calibration::EDGE_CONFIRM_US) tick();
	assert(getTelemetry().status == Status::Running);
	until(Status::EdgeStopping);
	assert(getTelemetry().pwmA == 0 && getTelemetry().pwmB == 0);
}
int main() {
	// 比例成立但并不靠近边缘，或比例仅等于阈值，都不能触发。
	fresh(); distances(200, 1000);
	for (int i=0; i<40; ++i) tick();
	assert(getTelemetry().status == Status::Running);
	distances(100, 300);
	for (int i=0; i<40; ++i) tick();
	assert(getTelemetry().status == Status::Running);
	// 换边重新计时；通信失败也不能累计靠边时长。
	distances(100, 400); for (int i=0; i<15; ++i) tick();
	distances(400, 100); for (int i=0; i<15; ++i) tick();
	assert(getTelemetry().status == Status::Running);
	devices::tof1.timeout = true; tick(); assert(runtime::edgeSide == 0);
	fresh(); trigger();
	until(Status::EdgeReverseSteering);
	assert(getTelemetry().appliedPhi == MAX_STEER && getTelemetry().pwmA == 0);
	until(Status::EdgeReversing);
	assert(getTelemetry().pwmA == calibration::EDGE_REVERSE_PWM);
	assert(levels[ports::IN_1] == (MOTOR_A_DIRECTION < 0));
	assert(levels[ports::IN_3] == (MOTOR_B_DIRECTION < 0));
	// 倒车后的距离决定新的前进方向，不能沿用旧的纠偏方向。
	distances(400, 100);
	until(Status::EdgeSettling);
	assert(getTelemetry().pwmA == 0 && getTelemetry().pwmB == 0);
	until(Status::EdgeForwardSteering);
	assert(getTelemetry().desiredPhi > 0 && getTelemetry().dTerm == 0);
	assert(getTelemetry().pwmA == 0);
	until(Status::WarmingUp); until(Status::Running);
	assert(runtime::edgeCooldown);
	for (int i=0; i<20; ++i) tick();
	assert(getTelemetry().status == Status::Running);
	// 右侧靠边时倒车舵角应镜像；中途传感器失效立即停机。
	fresh(); trigger(false); until(Status::EdgeReversing);
	assert(getTelemetry().appliedPhi == -MAX_STEER);
	devices::tof2.timeout = true; tick();
	assert(getTelemetry().status == Status::SensorFault && getTelemetry().pwmA == 0);
	// 调度中断不能让倒车输出继续维持。
	fresh(); trigger(); until(Status::EdgeReversing);
	nowUs += MAX_CONTROL_GAP_US + 1; update();
	assert(getTelemetry().status == Status::TimingFault && pwm[ports::ENA] == 0);
	// 前方障碍优先于退让动作。
	fresh(); trigger();
	echoUs = static_cast<unsigned long>(STOP_DISTANCE * .8f * 5800);
	for (unsigned i=0; i<OBSTACLE_CONFIRM_FRAMES; ++i) tick();
	assert(getTelemetry().status == Status::Obstacle && getTelemetry().pwmA == 0);
	// 持续靠边可超过旧的四次上限，每次仍经过冷却和重新确认。
	fresh();
	for (int attempt = 0; attempt < 6; ++attempt) {
		distances(100, 400);
		until(Status::EdgeStopping, 120);
		until(Status::EdgeReversing);
		until(Status::EdgeSettling);
		until(Status::EdgeForwardSteering);
		until(Status::WarmingUp);
		until(Status::Running);
		assert(runtime::edgeCooldown);
	}
	puts("Edge recovery trigger, reverse, resampling, interruption and limits passed");
}
