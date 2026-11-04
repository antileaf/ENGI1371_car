#include "control.h"
#include "runtime_debug.h"
#include <cassert>
#include <cstdio>
using namespace control;
void tick() { nowUs+=CONTROL_PERIOD_US; update(); }
void reset(uint32_t time=0) {
	nowUs=time; echoUs=5800;
	devices::tof1.mm=devices::tof2.mm=300;
	devices::tof1.timeout=devices::tof2.timeout=false;
	devices::tof1.last_status=devices::tof2.last_status=0;
	devices::tof1.waitUs=devices::tof2.waitUs=0;
	assert(start(50,60));
}
void obstacle() {
	echoUs=1000;
	for(int i=0;i<OBSTACLE_CONFIRM_FRAMES;++i) tick();
	assert(getTelemetry().status==Status::Obstacle);
	assert(getTelemetry().pwmA==0 && getTelemetry().pwmB==0);
}
int main() {
	reset();
	for(int i=1;i<HISTORY_SIZE;++i) { tick(); assert(getTelemetry().pwmA==0); }
	for(int i=0;i<40;++i) tick();
	assert(getTelemetry().status==Status::Running);
	assert(getTelemetry().pwmA==50 && getTelemetry().pwmB==60);
	assert(levels[ports::IN_1]==HIGH && levels[ports::IN_4]==HIGH);
	devices::tof2.last_status=1;
	tick();
	assert(getTelemetry().status==Status::SensorFault);
	assert(pwm[ports::ENA]==0 && pwm[ports::ENB]==0);
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
	devices::tof1.mm=400; devices::tof2.mm=200;
	float previous=0;
	for(int i=0;i<30;++i) {
		tick();
		assert(fabsf(getTelemetry().appliedPhi-previous)<=MAX_STEER_RATE*.061f);
		previous=getTelemetry().appliedPhi;
	}
	assert(getTelemetry().rawOffset>0 && previous<0);
	reset();
	echoUs=1000; tick(); tick();
	assert(getTelemetry().status!=Status::Obstacle);
	echoUs=0; tick(); assert(runtime::obstacleFrames==0);
	obstacle();
	echoUs=0; tick(); assert(getTelemetry().status==Status::WaitingToRestart);
	for(int i=0;i<10;++i) tick();
	echoUs=1000; tick(); assert(getTelemetry().status==Status::Obstacle);
	echoUs=0; tick();
	for(int i=0;i<34;++i) tick();
	assert(getTelemetry().status==Status::WarmingUp);
	assert(getTelemetry().pwmA==0 && getTelemetry().offsetRate==0);
	for(int i=0;i<35;++i) tick();
	assert(getTelemetry().pwmA==50 && getTelemetry().pwmB==60);
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
	assert(getTelemetry().status==Status::SensorFault && getTelemetry().pwmA==0);
	reset(); nowUs+=MAX_CONTROL_GAP_US+1; update();
	assert(getTelemetry().status==Status::TimingFault);
	reset(); devices::tof2.waitUs=MAX_CONTROL_GAP_US+1; tick();
	assert(getTelemetry().status==Status::TimingFault);
	assert(!start(0,50)); assert(!start(151,50));
	reset(0xffff0000UL); for(int i=0;i<35;++i) tick();
	assert(getTelemetry().status==Status::Running);
	stop(); tick(); assert(getTelemetry().status==Status::Stopped);
	nowUs+=1000000; runtime_debug::update();
	assert(Serial.output.str().find("offset_rate_mps=")!=std::string::npos);
	assert(Serial.output.str().find("encL_mps=")==std::string::npos);
	puts("ToF PD, PWM ramp, obstacle recovery/lock and fault checks passed");
}
