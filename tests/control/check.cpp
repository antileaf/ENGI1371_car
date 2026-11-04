#include "control.h"
#include <cassert>
void tick(int ticks=0){
 nowUs+=60000; encoder::detail::motorATicks+=ticks;encoder::detail::motorBTicks-=ticks;
 control::update();
}
void restart(){devices::tof1.timeout=false;devices::tof1.mm=300;devices::tof2.mm=300;devices::tof1.waitUs=0;echoUs=5800;assert(control::start(.12f));}
void detectObstacle(){echoUs=580;for(int i=0;i<control::OBSTACLE_CONFIRM_FRAMES;i++)tick();}
void warm(){for(int i=0;i<K;i++)tick();}
void stopped(control::Status reason){if(control::getTelemetry().status!=reason) fprintf(stderr,"status mismatch at %lu us: actual=%d expected=%d\n",static_cast<unsigned long>(nowUs),static_cast<int>(control::getTelemetry().status),static_cast<int>(reason));assert(control::getTelemetry().status==reason);assert(pwm[ports::PWMA]==0 && pwm[ports::PWMB]==0);}
int main(){
 assert(!control::start(NAN));stopped(control::Status::InvalidTarget);
#ifdef UNCALIBRATED
 assert(!control::start(.12f));stopped(control::Status::CalibrationMissing);
 puts("Missing calibration checks passed");
 return 0;
#else
 // 孤立近距离回波不停车，远距离和无回波都清零连续帧计数。
 restart();
 for(int i=0;i<control::OBSTACLE_CONFIRM_FRAMES-1;i++){echoUs=580;tick();assert(control::runtime::active);}
 echoUs=5800;tick();assert(control::runtime::obstacleFrames==0);
 for(int i=0;i<control::OBSTACLE_CONFIRM_FRAMES-1;i++){echoUs=580;tick();assert(control::runtime::active);}
 echoUs=0;tick();assert(control::runtime::obstacleFrames==0);
 for(int i=0;i<control::OBSTACLE_CONFIRM_FRAMES-1;i++){echoUs=580;tick();assert(control::runtime::active);}
 tick();stopped(control::Status::Obstacle);
 // 重新启动时不能沿用旧的确认计数。
 restart();echoUs=580;tick();assert(control::runtime::active);
 restart(); for(int i=0;i<K-1;i++){tick();stopped(control::Status::WarmingUp);}
 tick(); assert(control::getTelemetry().status==control::Status::LowSpeed);
 assert(control::getTelemetry().targetLeft>0);
 for(int i=1;i<=25;i++){tick(i<10?i*2:22);assert(control::runtime::active);}
 assert(control::getTelemetry().status==control::Status::Running);
 assert(devices::servo.angle==90);
 detectObstacle(); stopped(control::Status::Obstacle);
 echoUs=static_cast<unsigned long>((control::STOP_DISTANCE+control::RESUME_DISTANCE)*0.5f*5800); tick(); stopped(control::Status::Obstacle); // 两个阈值之间不满足恢复条件
 echoUs=5800; tick(); stopped(control::Status::WaitingToRestart);
 for(int i=0;i<20;i++){tick();stopped(control::Status::WaitingToRestart);}
 detectObstacle();stopped(control::Status::Obstacle);
 echoUs=5800;tick();stopped(control::Status::WaitingToRestart);
 for(int i=0;i<33;i++){tick();stopped(control::Status::WaitingToRestart);}
 tick();stopped(control::Status::WarmingUp);
 assert(control::runtime::sampleCount==0 && control::runtime::rampedSpeed==0);
 assert(control::runtime::requestedSpeed==.12f);
 warm();assert(control::getTelemetry().status==control::Status::LowSpeed);
 // 无回波也计入连续畅通时间，等待完成后恢复起步。
 detectObstacle();echoUs=5800;tick();echoUs=0;tick();
 stopped(control::Status::WaitingToRestart);
 assert(isnan(control::getTelemetry().frontDistance));
 for(int i=0;i<32;i++){tick();stopped(control::Status::WaitingToRestart);}
 tick();stopped(control::Status::WarmingUp);
 // 手动停车取消等待。
 restart();detectObstacle();echoUs=5800;tick();control::stop();
 for(int i=0;i<40;i++){tick();}
 stopped(control::Status::Stopped);
 // 长时间未轮询时，不能把盲区时间算作持续畅通。
 restart();detectObstacle();echoUs=5800;tick();
 nowUs+=3000000;control::update();stopped(control::Status::WaitingToRestart);
 for(int i=0;i<33;i++){tick();stopped(control::Status::WaitingToRestart);}
 tick();stopped(control::Status::WarmingUp);
 // 等待计时跨 micros() 回绕。
 nowUs=UINT32_MAX-1000000;restart();detectObstacle();echoUs=5800;tick();
 for(int i=0;i<33;i++){tick();stopped(control::Status::WaitingToRestart);}
 tick();stopped(control::Status::WarmingUp);
 restart(); devices::tof1.timeout=true;tick();stopped(control::Status::SensorFault);
 restart();echoUs=0;warm();assert(control::runtime::active);
 assert(control::getTelemetry().status==control::Status::LowSpeed);
 assert(isnan(control::getTelemetry().frontDistance));
 // 无回波行驶后连续收到近距离回波，确认后停车。
 detectObstacle();stopped(control::Status::Obstacle);
 restart();devices::tof1.mm=8190;tick();stopped(control::Status::SensorFault);
 restart();nowUs+=300000;control::update();stopped(control::Status::TimingFault);
 restart();devices::tof1.waitUs=210000;tick();stopped(control::Status::TimingFault);
 // 无运动超时检查已按调试需求关闭。
 restart();warm();for(int i=0;i<50;i++)tick();assert(control::runtime::active);
 // 不同通道计数验证 A/B 到实际左/右的映射，避免等速样本掩盖接反。
 restart();
 encoder::detail::motorATicks+=10;
 encoder::detail::motorBTicks-=20;
 tick();
 const auto &mapped = control::getTelemetry();
 assert(mapped.measuredLeft>0 && mapped.measuredRight>0);
 assert(control::MOTOR_A_IS_LEFT ? mapped.measuredLeft<mapped.measuredRight
                               : mapped.measuredLeft>mapped.measuredRight);
 restart();warm();devices::tof1.mm=200;devices::tof2.mm=400;tick();
 assert(control::TOF1_IS_LEFT ? control::getTelemetry().servoDegrees<90 : control::getTelemetry().servoDegrees>90);
 assert(control::TOF1_IS_LEFT ? control::getTelemetry().targetLeft>control::getTelemetry().targetRight
                            : control::getTelemetry().targetLeft<control::getTelemetry().targetRight);
 control::stop();stopped(control::Status::Stopped);
 uint32_t times[K]; float left[K],right[K],sl[K],sr[K];
 const uint32_t origin=UINT32_MAX-300000;
 for(int i=0;i<K;i++){
  uint32_t us=i*50000+i*i*1000; times[i]=origin+us;
  float t=us*1e-6f;
  float x=.01f+.002f*t+.001f*t*t;
  left[i]=.3f-x;right[i]=.3f+x;sl[i]=.19f;sr[i]=.21f;
 }
 state::CarState s={};assert(state::estimateTimedState(left,right,sl,sr,times,s));
 float t=(times[K-1]-times[0])*1e-6f;
 assert(fabsf(s.x-(.01f+.002f*t+.001f*t*t))<1e-5f);
 assert(fabsf(s.theta-(.002f+.002f*t)/.2f)<1e-4f);
 assert(fabsf(s.kappa-(.1f/.2f-.002f/.04f))<.001f);
 times[2]=times[1];assert(!state::estimateTimedState(left,right,sl,sr,times,s));
 float integral=0;for(int i=0;i<100;i++)control::speedPI(1,0,.06f,integral);
 assert(integral==0);assert(control::speedPI(0,.1f,.06f,integral)==0);
 // 不同采样间隔下均遵守角速度上限，左右反转也不能跳变。
 control::runtime::appliedPhi=0;
 float limited=control::limitSteeringRate(control::MAX_STEER,.06f);
 assert(fabsf(limited-control::MAX_STEER_RATE*.06f)<1e-6f);
 control::runtime::appliedPhi=limited;
 float reversed=control::limitSteeringRate(-control::MAX_STEER,.12f);
 assert(fabsf((limited-reversed)-control::MAX_STEER_RATE*.12f)<1e-6f);
 restart();devices::tof1.mm=200;devices::tof2.mm=400;warm();
 auto steeringData=control::getTelemetry();
 assert(fabsf(steeringData.appliedPhi)<=control::MAX_STEER_RATE*.061f);
 float ratio=(steeringData.targetRight-steeringData.targetLeft)
              /(steeringData.targetRight+steeringData.targetLeft);
 assert(fabsf(ratio-tanf(steeringData.appliedPhi)*W/(2*L))<1e-5f);
 restart();assert(control::runtime::appliedPhi==0);
 // 连续受阻满一分钟锁定，覆盖 micros() 回绕及恢复后的舵机状态。
 nowUs=UINT32_MAX-20000000UL;
 restart();detectObstacle();
 assert(devices::servo.attached());
 for(int i=0;i<999;i++){tick();stopped(control::Status::Obstacle);}
 tick();stopped(control::Status::ObstacleLocked);
 assert(!devices::servo.attached());
 const int writesAtLock=devices::servo.writes;
 echoUs=5800;
 for(int i=0;i<50;i++){tick();stopped(control::Status::ObstacleLocked);}
 assert(devices::servo.writes==writesAtLock && !devices::servo.attached());
 restart();assert(devices::servo.attached());
 // 无回波打断连续受阻，下一段需重新等待完整一分钟。
 detectObstacle();for(int i=0;i<500;i++)tick();
 echoUs=0;tick();stopped(control::Status::WaitingToRestart);
 echoUs=580;tick();stopped(control::Status::Obstacle);
 for(int i=0;i<999;i++){tick();stopped(control::Status::Obstacle);}
 tick();stopped(control::Status::ObstacleLocked);
 puts("Control simulation checks passed");
#endif
}



