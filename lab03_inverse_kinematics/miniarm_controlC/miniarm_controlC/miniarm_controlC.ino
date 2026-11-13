/*
  miniarm_controlC.ino  -  the Arduino half of miniarm_controlC.py
  ================================================================

  Upload this once with the Arduino IDE, close the Serial Monitor, then run
  miniarm_controlC.py. Python does all the math and calibration; this sketch only
  moves servos, reads the knobs, sets the LED, and reports back.

  It never reads or writes EEPROM, so the Hiwonder firmware's saved offsets and
  recorded motions are untouched. Re-upload the Hiwonder firmware any time.

  Libraries: Servo (built in) and FastLED 3.10.3 (newer FastLED versions clash
  with Servo: "multiple definition of __vector_11").

  Serial protocol: 115200 baud, one command per line
  ---------------------------------------------------
  Servo values are listed S1..S5: claw, wrist, forearm, arm, base,
  in the same raw numbers read_mini.ino prints.

  Python -> Arduino
    CFG lo1 hi1 lo2 hi2 lo3 hi3 lo4 hi4 lo5 hi5 stepMs settleMs
                     safe range of each servo, ramp speed, pause between joints
    INIT p1 p2 p3 p4 p5   first pose: servos switch on here (one quick snap)
    MODE A | K | I        APP, KNOB, or IK control (also sets the LED color)
    M p1 p2 p3 p4 p5      move all joints together
    J p1 p2 p3 p4 p5      move one joint at a time: base, arm, forearm, wrist, claw
    STOP                  freeze where the arm is right now

  Arduino -> Python
    READY                 sketch started (also after every reset)
    CFG OK                settings received
    MODE A | K | I        mode changed
    OK                    a move (INIT, M or J) has finished
    STOPPED               the arm was frozen
    K r1 r2 r3 r4 r5      knob readings, 0-1023, every 50 ms
    P p1 p2 p3 p4 p5      servo positions, every 100 ms while they change
    ERR <reason>          a command was not understood or not allowed
*/

#include <Servo.h>
#include <FastLED.h>

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
const long    BAUD = 115200;
const uint8_t SERVO_PINS[5] = { 7, 6, 5, 4, 3 };       // S1 claw ... S5 base
const uint8_t KNOB_PINS[5]  = { A0, A1, A2, A3, A4 };  // knob S1 ... S5
const uint8_t LED_PIN = 13;                            // the board's RGB LED

const uint8_t MOVE_ORDER[5] = { 4, 3, 2, 1, 0 };       // J moves: base, arm, forearm, wrist, claw

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
Servo servos[5];
CRGB leds[1];

bool servosOn = false;          // false until INIT: servos are limp
char mode = 'A';
int cur[5] = { 90, 90, 90, 90, 90 };    // where each servo is now
int tgt[5] = { 90, 90, 90, 90, 90 };    // where each servo is going
int lo[5]  = { 0, 0, 0, 0, 0 };         // safe range, from CFG
int hi[5]  = { 180, 180, 180, 180, 180 };
unsigned long stepMs = 15;              // ms per 1-unit step
unsigned long settleMs = 500;           // pause between joints in a J move

bool moveActive = false;        // an M or INIT move is running (send OK when done)
bool seqActive = false;         // a J move is running
uint8_t seqIndex = 0;           // which joint of MOVE_ORDER is moving
int seqTarget[5];
bool settling = false;
unsigned long settleStart = 0;

float knob[5];                  // smoothed knob readings, 0-1023
bool knobStarted = false;
bool positionsChanged = true;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
// Same convention as read_mini.ino: the claw channel (S1) is inverted
void writeServo(uint8_t i, int value) {
  servos[i].write(i == 0 ? 180 - value : value);
}

int clampToRange(uint8_t i, int value) {
  return constrain(value, lo[i], hi[i]);
}

// A knob reading 0-1023 -> a servo value inside that joint's safe range
int knobToServo(uint8_t i) {
  long raw = (long)(knob[i] + 0.5);
  return lo[i] + (raw * (hi[i] - lo[i]) + 511) / 1023;
}

void setLed(char m) {
  if (m == 'A')      leds[0] = CRGB(0, 160, 0);     // APP CONTROL: green
  else if (m == 'K') leds[0] = CRGB(255, 70, 0);    // KNOB CONTROL: orange
  else if (m == 'I') leds[0] = CRGB(0, 40, 255);    // IK CONTROL: blue
  else               leds[0] = CRGB(60, 60, 60);    // starting up: white
  FastLED.show();
}

void freeze() {                 // stop every joint where it is
  for (uint8_t i = 0; i < 5; i++) tgt[i] = cur[i];
  moveActive = false;
  seqActive = false;
  settling = false;
}

// Read up to n integers from text like "12 34 56"; returns how many were found
uint8_t readInts(char *text, int out[], uint8_t n) {
  uint8_t count = 0;
  char *end;
  while (count < n) {
    long v = strtol(text, &end, 10);
    if (end == text) break;
    out[count++] = (int)v;
    text = end;
  }
  return count;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
void handleLine(char *line) {
  int v[12];

  if (strncmp(line, "CFG ", 4) == 0) {
    if (readInts(line + 4, v, 12) != 12) { Serial.println(F("ERR CFG needs 12 numbers")); return; }
    for (uint8_t i = 0; i < 5; i++) {
      lo[i] = constrain(min(v[2 * i], v[2 * i + 1]), 0, 180);
      hi[i] = constrain(max(v[2 * i], v[2 * i + 1]), 0, 180);
    }
    stepMs = max(v[10], 1);
    settleMs = max(v[11], 0);
    Serial.println(F("CFG OK"));

  } else if (strncmp(line, "INIT ", 5) == 0) {
    freeze();
    if (readInts(line + 5, v, 5) != 5) { Serial.println(F("ERR INIT needs 5 numbers")); return; }
    for (uint8_t i = 0; i < 5; i++) {
      cur[i] = tgt[i] = clampToRange(i, v[i]);
      writeServo(i, cur[i]);                     // set the pulse first...
      if (!servosOn) servos[i].attach(SERVO_PINS[i]);  // ...then switch the servo on
    }
    servosOn = true;
    positionsChanged = true;
    Serial.println(F("OK"));

  } else if (strncmp(line, "MODE ", 5) == 0) {
    char m = line[5];
    if (m != 'A' && m != 'K' && m != 'I') { Serial.println(F("ERR MODE is A, K or I")); return; }
    freeze();                                    // nothing jumps when the mode changes
    mode = m;
    setLed(m);
    Serial.print(F("MODE ")); Serial.println(m);

  } else if (line[0] == 'M' || line[0] == 'J') {
    if (!servosOn)   { Serial.println(F("ERR send INIT first")); return; }
    if (mode == 'K') { Serial.println(F("ERR in knob mode")); return; }
    if (readInts(line + 1, v, 5) != 5) { Serial.println(F("ERR needs 5 numbers")); return; }
    freeze();
    if (line[0] == 'M') {
      for (uint8_t i = 0; i < 5; i++) tgt[i] = clampToRange(i, v[i]);
      moveActive = true;
    } else {
      for (uint8_t i = 0; i < 5; i++) seqTarget[i] = clampToRange(i, v[i]);
      seqIndex = 0;
      seqActive = true;
    }

  } else if (strcmp(line, "STOP") == 0) {
    freeze();
    Serial.println(F("STOPPED"));
    Serial.print('P');
    for (uint8_t i=0; i<5; i++) { Serial.print(' '); Serial.print(cur[i]); }
    Serial.println();

  } else if (line[0] != '\0') {
    Serial.print(F("ERR unknown command: ")); Serial.println(line);
  }
}

void readSerial() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) { buf[len] = '\0'; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Knobs: smooth every 10 ms, report every 50 ms
// ---------------------------------------------------------------------------
void updateKnobs() {
  static unsigned long lastRead = 0, lastSend = 0;
  if (millis() - lastRead >= 10) {
    lastRead = millis();
    for (uint8_t i = 0; i < 5; i++) {
      int raw = analogRead(KNOB_PINS[i]);
      knob[i] = knobStarted ? knob[i] * 0.7 + raw * 0.3 : raw;
    }
    knobStarted = true;
  }
  if (millis() - lastSend >= 50) {
    lastSend = millis();
    Serial.print('K');
    for (uint8_t i = 0; i < 5; i++) { Serial.print(' '); Serial.print((int)(knob[i] + 0.5)); }
    Serial.println();
  }
}

// ---------------------------------------------------------------------------
// Motion: one servo unit per stepMs, so every move is a slow ramp
// ---------------------------------------------------------------------------
bool stepToward(uint8_t i, int target) {        // returns true once the joint is there
  if (cur[i] != target) {
    cur[i] += (cur[i] < target) ? 1 : -1;
    writeServo(i, cur[i]);
    positionsChanged = true;
  }
  return cur[i] == target;
}

void updateMotion() {
  static unsigned long last = 0;
  if (!servosOn || millis() - last < stepMs) return;
  last = millis();

  if (mode == 'K') {                            // knobs drive the servos directly
    for (uint8_t i = 0; i < 5; i++) {
      tgt[i] = knobToServo(i);
      stepToward(i, tgt[i]);
    }

  } else if (seqActive) {                       // J: one joint at a time, bottom up
    if (settling) {
      if (millis() - settleStart < settleMs) return;
      settling = false;
    }
    while (seqIndex < 5 && cur[MOVE_ORDER[seqIndex]] == seqTarget[MOVE_ORDER[seqIndex]]) seqIndex++;
    if (seqIndex >= 5) {                        // every joint has arrived
      seqActive = false;
      Serial.println(F("OK"));
      return;
    }
    uint8_t j = MOVE_ORDER[seqIndex];
    tgt[j] = seqTarget[j];
    if (stepToward(j, tgt[j])) {                // this joint just arrived: pause, then the next
      settling = true;
      settleStart = millis();
    }

  } else {                                      // M: all joints together
    bool allThere = true;
    for (uint8_t i = 0; i < 5; i++) {
      if (!stepToward(i, tgt[i])) allThere = false;
    }
    if (moveActive && allThere) {
      moveActive = false;
      Serial.println(F("OK"));
    }
  }
}

void reportPositions() {
  static unsigned long last = 0;
  if (!servosOn || !positionsChanged || millis() - last < 100) return;
  last = millis();
  positionsChanged = false;
  Serial.print('P');
  for (uint8_t i = 0; i < 5; i++) { Serial.print(' '); Serial.print(cur[i]); }
  Serial.println();
}

// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(BAUD);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 1);
  FastLED.setBrightness(120);
  setLed('?');                                  // white until Python picks a mode
  Serial.println(F("READY"));
}

void loop() {
  readSerial();
  updateKnobs();
  updateMotion();
  reportPositions();
}
