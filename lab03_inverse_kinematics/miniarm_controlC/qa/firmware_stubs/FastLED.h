#pragma once
struct CRGB { int r,g,b; CRGB(int a=0,int c=0,int d=0):r(a),g(c),b(d){} };
const int WS2812=1, GRB=1;
struct FakeLED {
  template<int,int,int> void addLeds(CRGB*,int){}
  void setBrightness(int){} void show(){}
};
inline FakeLED FastLED;
