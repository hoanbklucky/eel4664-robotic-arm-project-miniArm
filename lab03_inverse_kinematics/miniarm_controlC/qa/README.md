# Host firmware checks

`firmware_timed_test.cpp` includes the actual Arduino sketch. The headers in
`firmware_stubs` replace only Servo/FastLED; the harness replaces clock, pins and
serial so assertions can exercise the real interpolation and command handlers.
These stubs are never used when compiling/uploading the sketch in Arduino IDE.

From `miniarm_controlC`:

```console
g++ -std=c++17 -Wall -Wextra -Werror -I qa/firmware_stubs qa/firmware_timed_test.cpp -o firmware_test
./firmware_test
```

The same build/run is in `test_timed_motion.py` when g++ is available. This does
not verify AVR resource usage, the Arduino toolchain, real pulse timing or motors.
