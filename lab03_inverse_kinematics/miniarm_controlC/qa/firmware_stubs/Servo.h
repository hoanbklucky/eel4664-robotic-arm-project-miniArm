#pragma once
#define MIN_PULSE_WIDTH 544
#define MAX_PULSE_WIDTH 2400
class Servo {
public:
  int pulse=1500; bool attached=false;
  void writeMicroseconds(int value){pulse=value;}
  void attach(int){attached=true;}
};
