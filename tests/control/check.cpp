#include "control.h"
#include "runtime_debug.h"
#include <cassert>
#include <cstdio>
using namespace control;
constexpr unsigned long OBSTACLE_ECHO_US = static_cast<unsigned long>(STOP_DISTANCE * .8f * 5800);
void tick() { nowUs+=CONTROL_PERIOD_US; update(); }
void reset(uint32_t time=0) {
	tof::firstInitialized=tof::secondInitialized=true;
	tof::firstFailures=tof::secondFailures=0;
	nowUs=time; echoUs=5800;
	devices::tof1.mm=devices::tof2.mm=300;
	devices::tof1.timeout=devices::tof2.timeout=false;
	devices::tof1.last_status=devices::tof2.last_status=0;
	devices::tof1.waitUs=devices::tof2.waitUs=0;
	assert(start(100,110));
}
void obstacle() {
	echoUs=OBSTACLE_ECHO_US;
	for(int i=0;i<OBSTACLE_CONFIRM_FRAMES;++i) tick();
	assert(getTelemetry().status==Status::Obstacle);
	assert(getTelemetry().pwmA==0 && getTelemetry().pwmB==0);
}
int main() {
	// 距离过远按上限参与控制，不触发通信故障、重初始化或降速。
	reset();
	devices::tof1.mm = devices::tof2.mm = static_cast<uint16_t>(calibration::TOF_MAX_DISTANCE_M * 1000);
	tick(); assert(getTelemetry().sensorIssues == 0);
	devices::tof2.mm += 1;
	for (int i=0; i<10; ++i) tick();
	assert(getTelemetry().sensorIssues == ToF2OutOfRange);
	assert(getTelemetry().status == Status::Running && getTelemetry().pwmA > 0 && getTelemetry().pwmB > 0);
	assert(fabsf(getTelemetry().rawOffset) < 1e-6f);
	assert(tof::secondInitialized && tof::secondFailures == 0);
	devices::tof1.mm = 300;
	devices::tof2.mm = 8000;
	tick();
	const float expectedOffset = (calibration::TOF_MAX_DISTANCE_M - .3f) / (calibration::TOF_MAX_DISTANCE_M + .3f);
	assert(fabsf(getTelemetry().rawOffset - (TOF1_IS_LEFT ? expectedOffset : -expectedOffset)) < 1e-5f);
	assert((TOF1_IS_LEFT ? getTelemetry().tofRight : getTelemetry().tofLeft) == 8.0f);
	devices::tof1.timeout = true; tick();
	assert(getTelemetry().sensorIssues == (ToF1Invalid | ToF2OutOfRange));
	assert(getTelemetry().pwmA == calibration::TOF_DEGRADED_PWM && getTelemetry().pwmB == calibration::TOF_DEGRADED_PWM);
	devices::tof1.timeout = false; devices::tof2.mm = 300; tick();
	assert(getTelemetry().sensorIssues == 0);
	reset();
	assert(start(1,1));
	for(int i=0;i<HISTORY_SIZE;++i) tick();
	assert(pwm[ports::ENA] >= calibration::START_PWM && pwm[ports::ENA] < calibration::MIN_DRIVE_PWM);
	for(int i=0;i<10;++i) tick();
	assert(pwm[ports::ENA]==calibration::MIN_DRIVE_PWM && pwm[ports::ENB]==calibration::MIN_DRIVE_PWM);
	stop();
	assert(pwm[ports::ENA]==0 && pwm[ports::ENB]==0);
	reset();
	for(int i=1;i<HISTORY_SIZE;++i) { tick(); assert(getTelemetry().pwmA==0); }
	for(int i=0;i<40;++i) tick();
	assert(getTelemetry().status==Status::Running);
	assert(getTelemetry().pwmA==100 && getTelemetry().pwmB==110);
	assert(levels[ports::IN_1]==HIGH && levels[ports::IN_4]==HIGH);
	devices::tof2.last_status=1;
	tick();
	assert(getTelemetry().status==Status::Running && getTelemetry().sensorIssues==ToF2Invalid);
	assert(pwm[ports::ENA]==calibration::TOF_DEGRADED_PWM && pwm[ports::ENB]==calibration::TOF_DEGRADED_PWM);
	devices::tof1.timeout=true; echoUs=5; tick();
	assert(getTelemetry().sensorIssues==(ToF1Invalid|ToF2Invalid|FrontInvalid));
	for(int i=0;i<OBSTACLE_CONFIRM_FRAMES;++i) tick();
	assert(getTelemetry().status==Status::Running);
	devices::tof1.timeout=false; devices::tof2.last_status=0; echoUs=5800;
	// 连续失败后需等待重初始化，再积累历史并完成起步斜坡。
	for(int i=0;i<80;++i) tick();
	assert(getTelemetry().sensorIssues==0 && getTelemetry().pwmB==110);
	devices::tof2.timeout=true;
	obstacle();
	assert(fabsf(getTelemetry().appliedPhi)<1e-6f);
	// 中心点仍有横移时必须纠偏；返回中心时 D 项与 P 项相反。
	assert(steeringDemand(0,.03f)<0);
	assert(steeringDemand(0,-.03f)>0);
	assert(steeringDemand(.04f,-.03f)>steeringDemand(.04f,0));
	assert(steeringDemand(0,0)==0);
	assert(steeringDemand(10,10)==-MAX_STEER);
	// 不等间隔的线性运动和时钟回绕，不依赖任何轮速输入。
	reset(0xffff0000UL);
	uint32_t time=nowUs; float position=-.02f;
	for(int i=0;i<35;++i) {
		uint32_t delta=i%2 ? 50000 : 70000; time+=delta;
		float dt=delta*1e-6f; position+=.04f*dt;
		observeOffset(position,time,dt);
	}
	assert(fabsf(getTelemetry().offsetRate-.04f)<.0001f);
	// 物理左侧更近意味着向左偏，应向右修正；变化率限幅和舵机限速。
	reset();
	devices::tof1.mm=TOF1_IS_LEFT ? 200 : 400; devices::tof2.mm=TOF1_IS_LEFT ? 400 : 200;
	float previous=0;
	for(int i=0;i<30;++i) {
		tick();
		assert(fabsf(getTelemetry().appliedPhi-previous)<=MAX_STEER_RATE*.061f);
		previous=getTelemetry().appliedPhi;
	}
	assert(getTelemetry().rawOffset>0 && previous<0);
	// 同样相差 100 mm，窄道应转得更多；交换左右后转角反向。
	auto steadyAngle = [](int left, int right) {
		reset();
		devices::tof1.mm = TOF1_IS_LEFT ? left : right;
		devices::tof2.mm = TOF1_IS_LEFT ? right : left;
		for (int i=0; i<30; ++i) tick();
		return getTelemetry().appliedPhi;
	};
	const float narrow = steadyAngle(150, 250);
	assert(getTelemetry().pwmA >= calibration::MIN_DRIVE_PWM && getTelemetry().pwmA < 100);
	// 当前增益可能使窄道纠偏达到满舵，外轮允许达到 PWM 上限。
	assert(getTelemetry().pwmB > 110 && getTelemetry().pwmB <= MAX_PWM);
	assert(fabsf(getTelemetry().rawOffset - 0.25f) < 1e-5f);
	const float wide = steadyAngle(350, 450);
	assert(narrow < wide && wide < 0);
	// 较宽道路仍应产生差速，但不应达到满舵或外轮 PWM 上限。
	assert(fabsf(wide) < MAX_STEER);
	assert(getTelemetry().pwmB > 110 && getTelemetry().pwmB < MAX_PWM);
	assert(fabsf(steadyAngle(250, 150) + narrow) < 1e-5f);
	assert(getTelemetry().pwmA > 100 && getTelemetry().pwmA <= MAX_PWM && getTelemetry().pwmB < 110);
	// 保持 4:1 满舵比例，但近侧不小于 0.2 m，单独验证差速而不触发退让。
	steadyAngle(200, 800);
	assert(getTelemetry().pwmB == MAX_PWM);
	steadyAngle(800, 200);
	assert(getTelemetry().pwmA == MAX_PWM);
	devices::tof1.timeout = true;
	tick();
	assert(getTelemetry().pwmA == calibration::TOF_DEGRADED_PWM && getTelemetry().pwmB == calibration::TOF_DEGRADED_PWM);
	reset();
	echoUs=OBSTACLE_ECHO_US; tick(); tick();
	assert(getTelemetry().status!=Status::Obstacle);
	echoUs=0; tick(); assert(runtime::obstacleFrames==0);
	obstacle();
	echoUs=0; tick(); assert(getTelemetry().status==Status::WaitingToRestart);
	for(int i=0;i<10;++i) tick();
	echoUs=OBSTACLE_ECHO_US; tick(); assert(getTelemetry().status==Status::Obstacle);
	echoUs=0; tick();
	for(int i=0;i<34;++i) tick();
	assert(getTelemetry().status==Status::WarmingUp);
	assert(getTelemetry().pwmA==0 && getTelemetry().offsetRate==0);
	for(int i=0;i<35;++i) tick();
	assert(getTelemetry().pwmA==100 && getTelemetry().pwmB==110);
	// 连续一分钟障碍锁定，移开不会重启，显式启动恢复舵机。
	obstacle();
	for(int i=0;i<1001;++i) tick();
	assert(getTelemetry().status==Status::ObstacleLocked && !devices::servo.enabled);
	echoUs=0; tick(); assert(getTelemetry().status==Status::ObstacleLocked);
	assert(start()); assert(devices::servo.enabled);
	// 观察中断不能计入连续受阻/畅通时间。
	reset(); obstacle(); nowUs+=MAX_CONTROL_GAP_US+1; tick();
	assert(runtime::blockedSinceUs==nowUs);
	reset(); devices::tof1.timeout=true; tick();
	assert(getTelemetry().sensorIssues==ToF1Invalid && getTelemetry().status==Status::WarmingUp && getTelemetry().pwmA==0);
	reset(); nowUs+=MAX_CONTROL_GAP_US+1; update();
	assert(getTelemetry().status==Status::TimingFault);
	reset(); devices::tof2.waitUs=MAX_CONTROL_GAP_US+1; tick();
	assert(getTelemetry().status==Status::TimingFault);
	assert(!start(0,50));
	assert(start(MAX_PWM, MAX_PWM));
	assert(!start(MAX_PWM + 1,50));
	assert(!start(50,MAX_PWM + 1));
	reset(0xffff0000UL); for(int i=0;i<35;++i) tick();
	assert(getTelemetry().status==Status::Running);
	stop(); tick(); assert(getTelemetry().status==Status::Stopped);
	nowUs+=1000000; runtime_debug::update();
	assert(Serial.output.str().find("offset_rate_per_s=")!=std::string::npos);
	assert(Serial.output.str().find("encL_mps=")==std::string::npos);
	// 障碍停车、等待重启和锁定期间继续刷新 ToF，不产生驱动输出。
	reset(); obstacle();
	const int stoppedAngle = devices::servo.angle;
	devices::tof1.mm = 450; devices::tof2.timeout = true; tick();
	assert(getTelemetry().sensorIssues & ToF2Invalid);
	assert((TOF1_IS_LEFT ? getTelemetry().tofLeft : getTelemetry().tofRight) > .44f);
	assert(getTelemetry().pwmA == 0 && getTelemetry().pwmB == 0 && devices::servo.angle == stoppedAngle);
	devices::tof2.timeout = false; echoUs = 0; tick();
	assert(getTelemetry().status == Status::WaitingToRestart && getTelemetry().sensorIssues == FrontNoEcho);
	devices::tof1.mm = 500; tick();
	assert((TOF1_IS_LEFT ? getTelemetry().tofLeft : getTelemetry().tofRight) == .5f);
	stop(Status::ObstacleLocked); servo::disable();
	devices::tof2.timeout = true;
	for (int i=0; i<50; ++i) tick();
	assert(tof::secondInitialized && !(getTelemetry().sensorIssues & ToF2Invalid));
	assert(getTelemetry().status == Status::ObstacleLocked && !devices::servo.enabled);
	assert(getTelemetry().pwmA == 0 && getTelemetry().pwmB == 0);
	puts("ToF PD, PWM ramp, obstacle recovery/lock and fault checks passed");
}
