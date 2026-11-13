// Host test of the ACTUAL sketch with only hardware/clock/serial stubbed.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cassert>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <iostream>
using std::min; using std::max;
template<class T> T constrain(T v,T lo,T hi){return min(hi,max(lo,v));}
#define F(x) x
const int A0=0,A1=1,A2=2,A3=3,A4=4;
unsigned long fakeMillis=0;
unsigned long millis(){return fakeMillis;}
int analogRead(int){return 512;}
struct FakeSerial {
  std::vector<std::string> lines; std::string current;
  void begin(long){} int available(){return 0;} char read(){return 0;}
  template<class T> void print(T value){std::ostringstream o;o<<value;current+=o.str();}
  void print(float value,int precision){std::ostringstream o;o<<std::fixed<<std::setprecision(precision)<<value;current+=o.str();}
  void println(){lines.push_back(current);current.clear();}
  template<class T> void println(T value){print(value);println();}
} Serial;
#include "../miniarm_controlC/miniarm_controlC.ino"
void command(const char *text){char line[128];std::strcpy(line,text);handleLine(line);}
bool near(float a,float b){return std::fabs(a-b)<0.002f;}
int main(){
  setup(); assert(!servosOn);
  assert(Serial.lines[0]=="FW 2.0.0 TIMED" && Serial.lines[1]=="READY");
  command("T 2000 40 110 130 100 50"); assert(Serial.lines.back()=="ERR send INIT first");
  command("CFG 40 90 0 180 0 180 0 180 0 180 15 0");
  command("INIT 40 90 90 90 90");assert(servosOn);
  int expectedClaw=(int)(MIN_PULSE_WIDTH+140.f*(MAX_PULSE_WIDTH-MIN_PULSE_WIDTH)/180.f+.5f);
  assert(servos[0].pulse==expectedClaw); // claw inverted exactly once
  fakeMillis=100;command("T 2000 40 110 130 100 50");assert(timedActive);
  fakeMillis=1100;updateMotion();
  assert(near(cur[0],40)&&near(cur[1],100)&&near(cur[2],110)&&near(cur[3],95)&&near(cur[4],70));
  fakeMillis=2100;updateMotion();assert(!timedActive&&Serial.lines.back()=="OK");
  assert(near(cur[1],110)&&near(cur[2],130)&&near(cur[4],50));
  fakeMillis=2200;command("T 2000 40 90 90 90 90");
  fakeMillis=2700;updateMotion();assert(near(cur[1],106.875)&&near(cur[2],123.75));
  float frozen[5];std::copy(cur,cur+5,frozen);int pulse=servos[1].pulse;
  command("STOP");assert(!timedActive&&servosOn);
  assert(Serial.lines[Serial.lines.size()-2]=="STOPPED");
  fakeMillis=15000;updateMotion();
  for(int i=0;i<5;i++)assert(near(cur[i],frozen[i]));
  assert(servos[1].pulse==pulse); // no snap/round-to-integer when stopped
  command("T 2000 40 90 90 90 181");assert(!timedActive);
  command("T 2000 40 nan 90 90 90");assert(!timedActive);
  command("T 500 40 0 0 0 0");assert(!timedActive); // peak-speed guard
  command("T 2000 40 90 90 90 90 garbage");assert(!timedActive);
  command("T 2000 40 90 90 90 90");assert(timedActive);
  fakeMillis=16000;updateMotion(); // resume from fractional stopped position
  assert(near(cur[1],(frozen[1]+90)/2));
  fakeMillis=17000;updateMotion();assert(!timedActive);
  command("MODE K");command("T 2000 40 90 90 90 90");assert(!timedActive);
  command("MODE A");command("INIT 40 90.25 90.25 90.25 90.25");
  command("J 40 91 91 91 91");
  for(int i=0;i<20 && seqActive;i++){fakeMillis+=20;updateMotion();}
  assert(!seqActive);for(int i=1;i<5;i++)assert(near(cur[i],91));
  std::cout<<"Firmware timed interpolation, common progress, STOP, guards, fractional resume and legacy J passed\n";
}
