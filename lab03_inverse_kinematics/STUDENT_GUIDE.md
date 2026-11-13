# Lab 03 — 3D inverse kinematics from simulation to miniArm

Use your DH model to predict XYZ, verify a 3×4 Jacobian, solve numerical IK,
and compare those predictions with the physical arm. Finish with a report
using [Lab03_Report_Template.docx](Lab03_Report_Template.docx).

## What you are expected to do

The repository includes a working reference implementation. Your work is to
derive and explain the model, independently verify the calculations, calibrate
your arm, measure its performance, and explain discrepancies. Submitting
screenshots or copying the supplied functions alone is insufficient.

Work in teams of three with one arm. Review Lab 02, Craig modified DH, partial
derivatives and numerical IK first. Do not assume that a successful simulation
establishes accurate physical motion.

## 1. Get the complete folder

Download the repository ZIP and extract it. Keep `matlab`, `python` and
`miniarm_controlC` together inside `lab03_inverse_kinematics`. The GUI imports
the shared solver from `../python`; copying only the GUI folder will not work.

| Folder or file | Role |
|---|---|
| `matlab/miniarm_model.m` | MATLAB DH table, geometry, limits and collision dimensions |
| `matlab/MiniArmIK.m` | MATLAB transformations, FK, Jacobians and DLS solver |
| `python/kinematics.py` | Shared Python implementation of those calculations |
| `python/miniarm_source_model.py` | Python default model matching MATLAB |
| `python/collision.py` | Capsule and table clearance; continuous joint-edge checks |
| `miniarm_controlC/miniarm_ik.py` | Unit conversion, calibrated limits and executable servo-path validation |
| `miniarm_controlC/miniarm_controlC.py` | XYZ GUI, preview, serial communication and waypoint queue |
| `miniarm_controlC/miniarm_calibration.py` | Measured angle ↔ servo conversion |
| `miniarm_controlC/miniarm_calibrate.py` | Calibration-entry form |
| `miniarm_controlC/miniarm_controlC/miniarm_controlC.ino` | Arduino servo, knob and LED control |

## 2. Establish the model in MATLAB

In MATLAB Online or desktop MATLAB, open `matlab` and run:

```matlab
verify_miniarm
verify_collision
demo_ik
demo_ik([-0.040; 0.100; 0.150])
```

MATLAB target inputs are **metres**. Base MATLAB is sufficient; the toolbox
comparison is optional. See the [MATLAB code guide](matlab/README.md) and
[expected screenshots](matlab/EXPECTED_OUTPUT.md).

Use these axes, viewed from behind the arm at the Arduino and knobs:
**X right, Y forward, Z up**. Base angle 0° points right, 90° forward, 180° left.
The positioning vector is `[base, shoulder, elbow, wrist]`; claw opening is
separate. Angles in the shared solver are **radians**. The GUI displays degrees.

Draw frames {0}–{4} and {T}, with all axes and positive rotations. Explain your
Craig modified-DH table, including the fixed tool offset. Default dimensions
are H=120, L1=56.5, L2=61.5, L3=85 mm; measure your own arm.

## 3. Understand and verify the 3D Jacobian

The position is the last column of the final FK matrix:
`p = T[0:3,3]`. Thus `J[0,0] = ∂x/∂q_base`, `J[1,0] = ∂y/∂q_base`,
and `J[2,0] = ∂z/∂q_base`. Repeat for the other joints to form all four columns.

For one column j, change **only** joint j, keeping the others fixed:

`J[:,j] ≈ (p(q + h*ej) - p(q - h*ej)) / (2*h)`.

Use h in radians. The relation is `Δp ≈ J Δq`; the velocity relation is
`p_dot = J q_dot`. The Python core's `partial` method differentiates each
joint's matrix analytically; `finite` uses the position differences above.

From a terminal in `miniarm_controlC`, run:

```console
python -m pip install -r requirements.txt
python -m unittest discover -s ../python -v
python -m unittest discover -s . -p test_integration.py -v
```

Inspect a pose independently:

```python
import numpy as np
import miniarm_ik as ik
G = dict(H=120., L1=56.5, L2=61.5, L3=85.)
q = [110., 70., -80., -40.]  # degrees for this display helper
J = ik.jacobian(q, G, 'partial')  # mm/radian
Jn = ik.jacobian(q, G, 'finite')
print(J)
print('Largest difference:', np.max(np.abs(J-Jn)))
```

Record J, Jn and their largest absolute difference. For a small change to one
joint, compare `J @ dq` with the actual FK position change. Convert dq to radians.
Explain why the base column's Z entry is zero for this idealized robot.

## 4. Run the Python GUI offline

Python needs NumPy, pyserial and Tkinter. Tkinter is included with the standard
Windows Python installer; on Linux install your distribution's Tk package.
Use the same Python interpreter for installation and execution.

```console
python miniarm_controlC.py --simulate
```

No Arduino connection is opened in this mode. If no `calibration.json` exists,
the GUI uses an explicitly labeled **synthetic calibration**. It is for software
testing only and is never saved as your measured calibration.

1. Click **Initialize at startup pose**. Offline this initializes the emulator.
2. Select **IK CONTROL**.
3. Enter X, Y, Z in **millimetres**, for example `-40`, `100`, `150`.
4. Click **Calculate / Preview**. Inspect the purple arm, red target, error curve,
   joint angles, solver residual and rounded-command error. This does not move.
5. Click **Move Arm** to execute the checked waypoints in the emulator.
6. Try the targets below, and save the experiment log.

| Trial | X mm | Y mm | Z mm | Purpose |
|---|---:|---:|---:|---|
| A | -40 | 100 | 150 | Left of center |
| B | 40 | 120 | 160 | Right of center |
| C | 0 | 130 | 130 | Center, different height |
| D | 1000 | 1000 | 1000 | Unreachable; no executable move |

A–C are candidate trials. Actual calibrated limits and starting posture can
cause failure. A local solver failure does not prove that no solution exists.
For hardware, record any rejected trial and use an instructor-approved nearby
replacement; include both the rejected and replacement coordinates in the report.

## 5. Trace one IK iteration

`Calculate / Preview` calls the adapter's `inverse_kinematics`, which calls
`python/kinematics.py:solve`. At each accepted current q:

1. FK calculates current XYZ; `e = target - position(m,q)`.
2. `jacobian(m,q)` calculates the 3×4 matrix at those angles.
3. DLS calculates `dq = J.T @ solve(J @ J.T + lambda**2 * I, e)`.
4. Limit the step size, then form `qCandidate = q + stepScale*dq`.
5. Apply calibrated joint limits to the candidate.
6. Accept it only if the actual FK error decreases and the configured
   collision/path checks pass. Otherwise halve the step and retry.
7. Set `q = trial`, save it, and recalculate FK and J at this new q.

The solver uses metres, lambda=0.01 m, maximum joint-vector step=0.10 rad,
0.0001 m convergence tolerance and at most 400 iterations, matching MATLAB.
The displayed error curve is against **solver updates, not elapsed time**.
The claw orientation is free; reaching XYZ does not guarantee a useful grasp.

## 6. Upload the Arduino sketch and calibrate

1. Open `miniarm_controlC/miniarm_controlC.ino` in Arduino IDE. Select your board
   and port. Use Servo and the lab's previously specified FastLED **3.10.3**.
   Upload, then close Serial Monitor. This sketch does not use EEPROM.
2. Set the measured `GEOMETRY` in `miniarm_controlC.py`.
3. Run `python miniarm_controlC.py --port COM11`, replacing COM11 with your port.
   Hardware mode does not silently fall back to simulation.
4. Connection leaves servos disabled. Read the displayed S1–S5 startup commands.
   With servo power off, arrange the arm near the corresponding startup pose
   before applying power and clicking **Initialize at startup pose**. INIT
   attaches servos at those commands immediately; it is not a checked motion
   from an unknown physical pose. Without calibration its commands are all 90.
5. In **APP CONTROL**, adjust the sliders and click **Send manual sliders**.
   Run `python miniarm_calibrate.py` alongside it. Measure five points per joint
   across the usable range, and enter safe min/max commands and claw settings.
6. Click **Check**, fix errors, Save, and **Reload calibration** in the GUI.
   Reloading requires explicit initialization again; it does not move the arm.

| Joint | Calibrator measurement | Conversion to mathematical joint angle |
|---|---|---|
| Base | 90° forward, increasing toward left | q_base = measured angle |
| Shoulder | 90° upright, increasing forward | q_shoulder = 180° − measured angle |
| Elbow | 90° aligned, increasing forward bend | q_elbow = 90° − measured angle |
| Wrist | 90° aligned, increasing forward bend | q_wrist = 90° − measured angle |

Measure link/base/gripper envelopes and review capsule radii and margin in
`python/miniarm_source_model.py`; keep the MATLAB model consistent. Defaults are
provisional. Camera, held object, gripper fingers and nearby equipment are not
fully represented. APP and KNOB controls bypass IK collision checking.

## 7. Validate FK and execute XYZ targets

For three distinct, clear poses, record mathematical joint angles, predicted
XYZ and measured XYZ relative to the same origin. Compute the Euclidean
position error `sqrt(dx**2 + dy**2 + dz**2)`; aim for **5 mm or less**.
Check frames, dimensions, calibration and measurement before blaming one cause.

In IK CONTROL, repeat Calculate / Preview then Move Arm for A–C or approved
replacement targets. Keep the first tests at Z ≥ 60 mm. Record iterations,
numerical residual, rounded-command miss, measured XYZ and measured target error.
The measured error goal is **10 mm or less**. Submit out-of-tolerance results
honestly with an explanation and corrective attempt.

The GUI preserves claw opening during IK, starts from Arduino's last reported
commanded pose, and queues one J waypoint at a time, waiting for OK. Each J
executes base → shoulder → elbow → wrist → claw. Python checks each raw-unit
step, including calibration knots, before enabling movement. The firmware
reports issued servo commands, not measured shaft angles. It cannot detect
stall, sag, slippage or a physical collision. STOP cancels the remaining queue
and asks Arduino to hold its current commands; it is a software stop.

Changing the target, calibration or mode requires a new preview. A reset or
communication failure requires re-establishing the pose and initialization.
Do not turn knobs during IK; knob movement triggers takeover and cancels the queue.

## 8. Pick and place

Choose an object and two positions your calibrated arm can reach with useful
claw orientations. Record their XYZ. Using IK CONTROL and the claw buttons:

1. Open the claw. Preview and move to ≥40 mm above the object.
2. Preview and move to grasp height; close the claw.
3. Lift, move above the destination, and lower, previewing each new target.
4. Open the claw and retreat upward.

Free claw pitch can prevent a successful grasp even when XYZ is accurate.
Document that limitation and any changed targets or object placement. Record
a video and save the log. Do not force a failed or obstructed trajectory.

## 9. Submit a report and supporting files

Submit `TeamName_Lab03_Report.docx` (PDF also if requested), your code changes,
`DH_table.txt`, your measured `calibration.json`, experiment log and video link.
Use the [report template](Lab03_Report_Template.docx). Include:

- Frames, dimensions, DH table and a labeled physical-arm sketch.
- MATLAB simulation screenshot and Python preview screenshot.
- 3×4 Jacobian, independent finite differences and a small-change experiment.
- One explained IK update showing current q, error, dq, trial and acceptance.
- Calibration evidence, three FK measurements and three physical IK trials.
- Unreachable-target rejection, pick-and-place video and limitations discussion.

Do not invent measurements. Label unperformed hardware work explicitly.
Keep your measured calibration private to your team; do not replace it with the
synthetic offline mapping. The public repository supplies complete reference
code rather than a hidden-solution student generator.

## Troubleshooting

| Symptom | Action |
|---|---|
| `No module named serial` or `numpy` | Install requirements with the Python used to launch the GUI |
| Cannot find `kinematics` | Restore sibling `../python` folder; download the complete repository |
| Port access denied | Close Serial Monitor and other GUI instances; check the port |
| Window does not initialize automatically | Expected; arrange startup pose and click Initialize |
| Move Arm disabled | Select IK mode, initialize, load calibration, then Calculate / Preview |
| Starting pose collides | Inspect the physical pose/model; establish a clear starting pose manually |
| IK converges but execution path rejected | Sequential rounded servo motion differs from continuous solver motion; try a closer target or a clear starting posture |
| Solver residual tiny, measured error large | Check calibration, dimensions, reference origin, backlash and load |
| Target works offline but fails on hardware | Actual calibrated limits differ from the synthetic mapping |

Further details: [MATLAB guide](matlab/README.md),
[Jacobian walkthrough](matlab/JACOBIAN_WALKTHROUGH.md),
[collision model](COLLISIONS.md).
