# Python GUI and Arduino quick start

Read the complete [student lab instructions](../STUDENT_GUIDE.md), including
calibration, XYZ axes, startup and report requirements.

From this folder:

```console
python -m pip install -r requirements.txt
python miniarm_controlC.py --simulate
```

Simulation opens automatically at the upright reference in **IK CONTROL**.
Enter XYZ mm, click **Calculate / Preview**, then **Move Arm**. No startup
button click is required; **Straighten arm upright** is an optional later
return-to-reference action. Offline mode opens no serial
port. By default it uses synthetic calibration and ignores local calibration.json;
that synthetic mapping is not a measured hardware calibration. Your file is not
changed. To test the saved hardware calibration and limits in the emulator:

```console
python miniarm_controlC.py --simulate --use-calibration
```

The banner distinguishes synthetic and saved-calibration modes. Reload in
synthetic mode explains how to switch; it cannot silently load a saved file.
Hardware mode continues to use the saved calibration. A purple preview can also
show a failed solver attempt: check its status/residual. Move Arm is enabled
only after successful IK and checked servo-path preparation.

The preview starts in **Rear right (Arduino side)** view, looking from above.
Use the Preview camera menu for Rear, Top or Right side. The colored arrows are
world frame {0}: X right, Y forward, Z up. Gray dashed links are the current
estimate, purple links are the calculated preview and the red x is the target.
The current readout changes only as the emulator/arm command sequence advances.
See the [student guide](../STUDENT_GUIDE.md#5-run-the-python-gui-offline)
for the reference drawing and interpretation.

For hardware, upload `miniarm_controlC/miniarm_controlC.ino`, close Serial
Monitor, then run `python miniarm_controlC.py --port COM11` with your actual port.
Use `python miniarm_calibrate.py` to create your measured `calibration.json`.
Hardware still requires explicit first initialization. With calibration,
**Straighten arm upright** uses q=[90,90,0,0] degrees
(base forward, shoulder vertical, elbow/wrist aligned), independently of the
sliders or IK seed. First enable uses immediate INIT, so arrange the unloaded
arm near upright using the instructor's procedure before clicking. After
initialization the button performs a checked gradual move and preserves the
claw command. If upright is outside calibrated limits or its path is blocked,
it reports the reason instead of substituting a different pose. Before any
calibration the button is **Enable raw neutral (uncalibrated)**: raw 90 on
all channels has unknown physical angles and is for supervised calibration.

Read [why and how to calibrate](../CALIBRATION_GUIDE.md), including the
startup-pose instructions, before initializing real servos. Targets stay XYZ;
calibration pairs an applied raw command with an independently measured DH
joint angle. The form's **Why calibrate? / How to measure** button explains
the references and gives a worked example. No polar workspace paper is required.

New calibration files store DH angles directly (`angle_convention=joint`):
elbow/wrist 0 degrees means aligned, with downward bending negative. Older
files remain readable and are converted for display while preserving their
command mapping. Update `miniarm_calibrate.py`, `miniarm_calibration.py` and
`miniarm_ik.py` together; keep your own measured `calibration.json`.

Keep this folder next to `../python`. The solver is shared with the existing
Python reference, which matches the MATLAB implementation. The old planar
Angle/Distance/Height API is replaced by XYZ; do not mix files from old copies.

The default simulation now uses the same joint limits as MATLAB: base/shoulder
0..180 degrees, elbow/wrist -90..90 degrees. Its synthetic servo mappings
are invented to cover those limits; they are not hardware calibration values.
See the [pipeline audit](../PYTHON_MATLAB_AUDIT.md) for DH/frame definitions,
calibration equations, command ordering and verification results.

## Tests

```console
python -m unittest discover -s ../python -v
python -m unittest discover -s . -p test_integration.py -v
python -m unittest discover -s . -p "test_*.py" -v
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


The control sliders are labeled **Base, Shoulder, Elbow, Wrist, Claw** to name
the joints consistently with the DH-angle readout and calibration form. Existing
JSON keys `arm` (shoulder) and `forearm` (elbow) are retained for compatibility;
existing calibration files need no renaming.

## Active mode, startup lock and STOP

The active mode is highlighted (APP green, KNOB orange, IK blue) and named in
text. On hardware, mode/motion controls stay disabled until startup initialization
is acknowledged. STOP and Reload calibration stay available; simulation still
opens ready in IK mode. Controls are disabled during a move or pending STOP.

STOP immediately cancels Python's future waypoints, then sends a software stop
before redrawing. Arduino freezes its command ramp when it processes the command;
servos remain powered to hold the last commanded positions. It is not a guaranteed
instantaneous emergency stop: serial delay and mechanical settling still apply.
The GUI waits for STOPPED and the next position report before unlocking controls.
In KNOB mode it exits to APP, and knob takeover cannot restart motion after a user
STOP until another explicit mode/motion action. The old IK plan is discarded.

## Predictable upright and clearer feedback

After initialization, **Straighten arm upright** is a checked move that preserves
APP or IK mode and the claw command. It does not silently select APP. In KNOB
mode it is disabled; the next-step hint explains that APP or IK must be selected
first so live knob control does not immediately undo the upright pose.

The interface groups raw manual servo commands separately from XYZ targets.
A next-step line explains the available action and disabled controls. A visible
result line reports successful previews, solver failures, rounding/path rejection
and invalid XYZ fields. Enter in an XYZ field runs Calculate / Preview; it never
sends movement. Only actual XYZ edits invalidate a preview, not navigation keys.
The selected mode is still highlighted and hardware initialization stays gated.

The calibration window shows unsaved edits, a Check/Save summary and a selectable
file path. It asks before closing or reloading would discard unsaved measurements;
a failed validation/save keeps the window and entries intact. It is resizable,
with the diagnostic area expanding. Saving still writes DH joint-angle mode.
