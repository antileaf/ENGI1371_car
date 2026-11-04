#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
constexpr int A0=14,A1=15,A2=16,A3=17,A4=18,A5=19,SDA=18,SCL=19;
constexpr int LOW=0,HIGH=1,INPUT=0,OUTPUT=1,INPUT_PULLUP=2,RISING=3,NOT_AN_INTERRUPT=-1;
uint32_t nowUs=0;
unsigned long echoUs=5800;
int levels[32]={}, pwm[32]={};
unsigned long micros(){return nowUs;}
unsigned long millis(){return nowUs/1000;}
void delay(unsigned long ms){nowUs+=ms*1000;}
void delayMicroseconds(unsigned int us){nowUs+=us;}
void digitalWrite(int pin,int value){levels[pin]=value;pwm[pin]=0;}
int digitalRead(int pin){return levels[pin];}
void analogWrite(int pin,int value){pwm[pin]=value;}
void pinMode(int,int){}
void noInterrupts(){} void interrupts(){}
int digitalPinToInterrupt(int pin){return pin==2?0:pin==3?1:-1;}
void attachInterrupt(int,void(*)(),int){}
unsigned long pulseIn(int,int,unsigned long){return echoUs;}
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
#include <sstream>
#define F(text) text
struct SerialMock {
 std::string input;
 int available(){return static_cast<int>(input.size());}
 int read(){if(input.empty()) return -1; int c=input.front(); input.erase(0,1); return c;}
 std::ostringstream output;
 void begin(int){}
 template<class T> void print(T value){output<<value;}
 void print(float value,int){output<<value;}
 template<class T> void println(T value){print(value);println();}
 void println(){output<<'\n';}
} Serial;

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
