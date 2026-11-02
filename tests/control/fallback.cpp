#include "control.h"
#include <cassert>

void frame(int a, int b) {
    nowUs += 60000;
    encoder::detail::motorATicks += a;
    encoder::detail::motorBTicks -= b;
    control::update();
}

int main() {
    speed_fallback::Wheel wheel;
    // 三帧同 PWM 的有效反馈建立比例，随后连续缺失才切换。
    for (int i=0;i<3;++i) wheel.observe(.1f,50,.1f,.1f);
    assert(fabsf(wheel.learnedGain-.002f)<1e-6f);
    wheel.observe(0,50,.1f,.1f);assert(!wheel.active);
    wheel.observe(0,50,.1f,.1f);assert(!wheel.active);
    const float estimated=wheel.observe(0,50,.1f,.1f);
    assert(wheel.active && wheel.referenceMeasured && fabsf(estimated-.1f)<1e-6f);
    assert(wheel.openLoopPwm(.2f,150)==100);
    assert(wheel.openLoopPwm(1,150)==150);
    assert(wheel.openLoopPwm(0,150)==0);
    wheel.observe(.1f,50,.1f,.1f);assert(wheel.active);
    wheel.observe(0,50,.1f,.1f);assert(wheel.active); // 单帧恢复不切回
    for(int i=0;i<3;++i) wheel.observe(.1f,50,.1f,.1f);
    assert(!wheel.active);
    wheel = speed_fallback::Wheel{};
    for(int i=0;i<3;++i) wheel.observe(0,40,.08f,.1f);
    assert(wheel.active && !wheel.referenceMeasured);
    assert(wheel.openLoopPwm(.12f,150)==60);
    assert(wheel.observe(-.2f,40,.08f,.1f)==-.2f && !wheel.active);
    wheel = speed_fallback::Wheel{};
    for(int i=0;i<20;++i) wheel.observe(0,0,0,.1f);
    assert(!wheel.active);

    assert(control::start(.12f));
    for(int i=0;i<K;++i) frame(0,0);
    // 两侧完全无计数：均切换估算，不以零速度持续积累 PI。
    for(int i=0;i<30;++i) frame(0,0);
    assert(control::runtime::active);
    assert(control::getTelemetry().fallbackLeft && control::getTelemetry().fallbackRight);
    assert(control::getTelemetry().encoderLeft==0 && control::getTelemetry().encoderRight==0);
    assert(control::getTelemetry().measuredLeft>0 && control::getTelemetry().measuredRight>0);
    assert(control::getTelemetry().car.kappa==0 && control::getTelemetry().car.alpha==0);
    assert(control::getTelemetry().pwmLeft<=control::MAX_PWM);
    // 单独恢复 motor B，不应影响 motor A 的 fallback。
    for(int i=0;i<3;++i) frame(0,20);
    const auto &data=control::getTelemetry();
    assert(control::MOTOR_A_IS_LEFT ? data.fallbackLeft && !data.fallbackRight
                                  : !data.fallbackLeft && data.fallbackRight);
    for(int i=0;i<3;++i) frame(20,20);
    assert(!data.fallbackLeft && !data.fallbackRight);
    assert(control::runtime::encoderOnlySamples<K); // 历史仍未完全摆脱估算值
    control::stop();
    assert(!control::runtime::fallbackLeft.active && !control::runtime::fallbackRight.active);
    assert(control::start(.12f));
    assert(!control::getTelemetry().fallbackLeft && !control::getTelemetry().fallbackRight);
    puts("Dual encoder fallback checks passed");
}
