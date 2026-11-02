#pragma once
struct Servo {
 int angle=90, writes=0;
 bool enabled=false;
 void attach(int){enabled=true;}
 void detach(){enabled=false;}
 bool attached(){return enabled;}
 void write(int value){angle=value;++writes;}
};
