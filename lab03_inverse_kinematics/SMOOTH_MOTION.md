# Smooth final-target motion — v2.0.0, Ngo Lab

The control GUI now displays a prominent **SIMULATION — NO PHYSICAL MOVEMENT**
or **REAL ROBOT** banner, application version, developer (Ngo Lab), and
firmware/emulator version. The calibration editor displays the same app version
and clearly says that it does not connect to or move the robot.

## Update both sides

1. Download the latest complete lab folder; retain your measured calibration.json.
2. Upload `miniarm_controlC/miniarm_controlC/miniarm_controlC.ino` using the board,
   port and previously specified Servo/FastLED libraries. Close Serial Monitor.
3. Start Python normally with your hardware port, or use `--simulate` offline.
4. Confirm **v2.0.0** and **Firmware 2.0.0** (or Emulator 2.0.0) in the GUI.
   An older/unknown sketch cannot enable the new timed Move command: upload the
   updated sketch rather than mixing firmware and Python versions.
5. Hardware still requires the explicit first-enable upright procedure.
   Simulation still opens initialized in IK mode. Existing joint-mode calibration
   files and their measurements remain compatible; the DH table is unchanged.

## Why the motion changed

IK iterations are a numerical search for joint angles. They are not samples of
a time trajectory. Previously, Python converted many accepted IK iterations into
separate J moves. Each J moved one servo at a time, and each waypoint needed an
acknowledgement before the next one. That can introduce stops, uneven pacing and
an unnatural route through the numerical search history.

Now Python solves IK, converts **only its final joint configuration** through
calibration, rounds the four positioning targets, preserves the claw command,
and checks a direct coordinated path. Move Arm sends **one final-target packet**.
Arduino generates intermediate servo commands locally and moves all channels
with the same progress parameter. The error curve still plots IK iterations;
it is not elapsed movement time and those iterations are not transmitted.

## Interpolation and timing

For duration D and elapsed fraction u=t/D clipped to [0,1], firmware uses:

`s(u) = 3*u^2 - 2*u^3`

`raw_i(t) = raw_start_i + s(u)*(raw_final_i - raw_start_i)`

Every channel starts together and reaches its final value together. The profile
has zero command velocity at its endpoints, providing easing at start and finish.
This is a coordinated path in calibrated **raw servo-command space**, not a
straight Cartesian XYZ path. With nonlinear piecewise calibration, mathematical
joint angles follow the corresponding calibrated curve. No IK is run on Arduino.

The GUI's **Move time (s, minimum)** defaults to 2 seconds. Larger values slow
the move. The software may extend it so the cubic profile's peak command speed
is at most 45 raw units/s (a command-space limit, not a measured shaft-speed or
universal mechanical safety rating). The preview displays the resulting duration.
Timing edits invalidate the old preview, so calculate again before moving.

Arduino updates the profile about every 20 ms without blocking its loop.
It sends sub-degree pulses with Servo.writeMicroseconds using the default Servo
endpoint mapping, preserving the same calibrated raw 0..180 convention and the
single S1/claw inversion. Reported raw positions retain three decimal places;
they remain command estimates, not encoder measurements. Legacy manual J/M
commands remain available; normal IK and calibrated upright/claw actions use T.
First INIT still attaches at its reference immediately, rather than interpolating
from an unknown physical starting pose.

## The actual path is checked

A collision-free IK solution does not guarantee a safe direct move from the
current pose. Python checks raw limits, nominal/calibrated joint limits, modeled
capsule/table clearance along the coordinated path, and the predicted XYZ miss
after final command rounding. The path is split wherever **any** channel crosses
a calibration knot. Within each segment, q is affine in common raw progress, so
the existing continuous joint-edge collision check applies. Cubic easing changes
only timing, not the geometric path.

If that direct path fails, Move Arm stays disabled and the result message asks
for an intermediate XYZ target. The program does not silently use the numerical
IK history as a fallback trajectory or disable collision checks. Capsule geometry
is still approximate; servo lag, load, camera and surrounding obstacles are not
fully modeled. Physical smoothness/accuracy must be evaluated on the arm.

## Protocol and STOP

The updated sketch announces `FW 2.0.0 TIMED` before `READY`. A timed command is:

```text
T duration_ms S1 S2 S3 S4 S5
```

For example, `T 2000 70 90 90 90 90` requests those raw targets over 2 seconds;
those numbers are an illustration, not a universal hardware upright prescription.
Duration must be an integer 500..25000 ms. Firmware rejects invalid/nonfinite,
out-of-range or too-fast requests. It returns one OK when interpolation ends.
P reports can contain fractional raw values. Update the Python code with the
sketch; older code that parses P as integers cannot read this protocol correctly.

STOP remains responsive to the main loop during interpolation. It freezes the
last commanded fractional values without detaching or removing servo power.
Python cancels its plan and waits for STOPPED plus a P report before unlocking
controls. It exits KNOB mode first, as before. STOP is not a guaranteed
instantaneous emergency stop; USB latency and mechanical settling still apply.

## Verification

From `miniarm_controlC`:

```console
python -m unittest discover -s ../python -v
python -m unittest discover -s . -p "test_*.py" -v
```

**73 tests passed; 2 native Tk tests were skipped without a display.** New tests
cover one-packet execution despite many IK iterations, common cubic progress,
finite/timing limits, fractional STOP/resume, calibration-knot splitting, safe
endpoints with an unsafe direct path, and rejection of older firmware capabilities.
A host C++ harness compiles and runs the actual sketch with stubbed clock,
serial and hardware libraries; it checks endpoints, STOP, speed/range/parser guards,
claw inversion and fractional legacy stepping. This is not an AVR/Arduino IDE
build or a physical servo test. Upload/compile on the actual board is still needed.
