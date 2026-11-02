#include "control.h"
#include <cassert>
#include <cstdio>
int main() {
 using namespace control;
 assert(steeringCalibrationReady());
 const bool increasing = steeringCalibration[4].servoDeg > steeringCalibration[0].servoDeg;
 float last = steeringCalibration[0].servoDeg;
 for (int i=0;i<5;++i) {
  float out;
  assert(interpolateSteering(steeringCalibration[i].wheelDeg,true,out));
  assert(out==steeringCalibration[i].servoDeg);
  assert(interpolateSteering(out,false,out));
  assert(out==steeringCalibration[i].wheelDeg);
 }
 for (int i=0;i<4;++i) {
  const auto &a=steeringCalibration[i]; const auto &b=steeringCalibration[i+1];
  for(int j=1;j<=100;++j) {
   float x=a.wheelDeg+(b.wheelDeg-a.wheelDeg)*j/100, y, recovered;
   assert(interpolateSteering(x,true,y));
   assert(y>=fminf(a.servoDeg,b.servoDeg) && y<=fmaxf(a.servoDeg,b.servoDeg));
   assert(increasing ? y>=last : y<=last); last=y;
   assert(interpolateSteering(y,false,recovered));
   assert(fabsf(recovered-x)<.003f);
  }
 }
 for(int i=1;i<4;++i) {
  float x=steeringCalibration[i].wheelDeg, a,b,c;
  const float step=.01f;
  assert(interpolateSteering(x-step,true,a));
  assert(interpolateSteering(x,true,b));
  assert(interpolateSteering(x+step,true,c));
  assert(fabsf((b-a)/step-(c-b)/step)<.03f);
 }
 float out;
 assert(!interpolateSteering(-31,true,out) && isnan(out));
 assert(!interpolateSteering(31,true,out) && isnan(out));
 assert(!interpolateSteering(NAN,true,out) && isnan(out));
 assert(!interpolateSteering(181,false,out) && isnan(out));
#ifdef KNOWN_MIDPOINT
 assert(interpolateSteering(-22.5f,true,out));
 assert(fabsf(out-61.458333f)<.0001f);
#endif
 puts("PCHIP checks passed");
}
