#include "hardware_test.h"
#include <cassert>
int main() {
 assert(encoder::setup());
 encoder::resetSpeedWindow();
 const auto before=encoder::readCounts();
 encoder::detail::motorATicks+=10;
 encoder::detail::motorBTicks-=20;
 nowUs+=60000;
 float a,b;
 assert(encoder::readSpeeds(a,b));
 assert(a>0 && b>0 && fabsf(b-2*a)<1e-6f);
 const auto after=encoder::readCounts();
 assert(encoder::countDelta(after.motorA,before.motorA)==10);
 assert(encoder::countDelta(after.motorB,before.motorB)==-20);
 // 倒转仍应给出负速度，不使用绝对值掩盖方向。
 encoder::detail::motorATicks-=10;encoder::detail::motorBTicks+=20;
 nowUs+=60000;
 assert(encoder::readSpeeds(a,b) && a<0 && b<0);
 hardware_test::setup();
 assert(devices::servo.angle==hardware_test::SERVO_ANGLE);
 Serial.input="12"; hardware_test::update();
 assert(devices::servo.angle==hardware_test::SERVO_ANGLE);
 Serial.input="0\r\n"; hardware_test::update();
 assert(devices::servo.angle==120);
 Serial.input="181\n-1\n9x0\n"; hardware_test::update();
 assert(devices::servo.angle==120);
 Serial.input="0\n"; hardware_test::update(); assert(devices::servo.angle==0);
 Serial.input="180\n"; hardware_test::update(); assert(devices::servo.angle==180);
 nowUs+=250000;
 encoder::detail::motorATicks+=10;
 encoder::detail::motorBTicks-=5;
 hardware_test::update();
 const auto output=Serial.output.str();
 assert(output.find("encA=10")!=std::string::npos);
 assert(output.find("encB=-5")!=std::string::npos);
 assert(output.find("dA=10")!=std::string::npos);
 assert(output.find("v_motor_a=")!=std::string::npos);
 assert(output.find("v_motor_b=")!=std::string::npos);
 assert(output.find("v_left=")==std::string::npos);
 nowUs+=250000;devices::tof1.timeout=true;hardware_test::update();
 assert(Serial.output.str().find("tof1=TIMEOUT")!=std::string::npos);
 puts("Motor A/B encoder signs and test output checks passed");
}
