# Python GUI and Arduino quick start

Read the complete [student lab instructions](../STUDENT_GUIDE.md), including
calibration, XYZ axes, startup and report requirements.

From this folder:

```console
python -m pip install -r requirements.txt
python miniarm_controlC.py --simulate
```

Click **Initialize at startup pose**, select **IK CONTROL**, enter XYZ mm,
click **Calculate / Preview**, then **Move Arm**. Offline mode opens no serial
port. Its fallback synthetic calibration is not a measured hardware calibration.

For hardware, upload `miniarm_controlC/miniarm_controlC.ino`, close Serial
Monitor, then run `python miniarm_controlC.py --port COM11` with your actual port.
Use `python miniarm_calibrate.py` to create your measured `calibration.json`.
Read the startup-pose instructions before initializing real servos.

Keep this folder next to `../python`. The solver is shared with the existing
Python reference, which matches the MATLAB implementation. The old planar
Angle/Distance/Height API is replaced by XYZ; do not mix files from old copies.

## Tests

```console
python -m unittest discover -s ../python -v
python -m unittest discover -s . -p test_integration.py -v
```

The latter opens a withdrawn Tk window and tests preview/execution using an
emulator. It requires a desktop-capable Python/Tk environment, but no Arduino.

## Execution details

The solver enforces measured joint limits and configured capsule clearance.
Accepted IK configurations become rounded servo waypoints. Each Arduino `J`
leg moves one joint at a time; Python checks that exact raw-command path,
including piecewise calibration knots. It waits for each `OK` before sending
the next waypoint. The claw command is preserved during IK.

`P` and `OK` describe commands issued by firmware, not measured physical motion.
Manual APP/KNOB controls bypass IK checks. Capsule dimensions remain provisional
until measured, and do not fully represent camera, fingers, payload or obstacles.

Full reference code is provided for students to inspect, verify and explain.
The old solution-removal generator is not used with this shared implementation.
