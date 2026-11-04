#pragma once
#include "Arduino.h"
struct VL53L0X{
 int last_status=0;
 uint16_t mm=300; bool timeout=false; uint32_t waitUs=0;
 uint16_t readRangeContinuousMillimeters(){nowUs+=waitUs;return mm;}
 bool timeoutOccurred(){return timeout;}
 void setTimeout(int){} bool init(){return true;}
 void setAddress(int){} void startContinuous(int){}
};
