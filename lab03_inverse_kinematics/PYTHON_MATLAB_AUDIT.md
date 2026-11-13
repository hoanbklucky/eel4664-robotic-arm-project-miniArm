# MATLAB → Python GUI → calibrated servo commands

Yes: the XYZ target and mathematical joint angles have exactly the same meaning
in MATLAB and Python. Calibration converts those joint angles into raw commands;
it must not redefine the frames, change the DH table, or add another base-angle
shift. This audit compares the repository's current MATLAB source with all
Python modules in this lab, plus the Arduino command protocol.

## 1. Common frames and Craig modified DH

World frame {0}: origin at the bottom of the modeled base post, on the table;
X right, Y forward, Z up, viewed from the rear Arduino/knob side. This is a
right-handed frame. Mathematical q order is **base, shoulder, elbow, wrist**.
Base q=0 points right; q=90 degrees points forward; q=180 points left.
Shoulder q=0 is horizontal forward, q=90 upright. Elbow and wrist q=0 mean
aligned with the previous link's forward extension. Negative relative pitch
bends downward in the usual forward-reaching pose. Claw opening is separate.

| Row i | a_(i−1), m | alpha_(i−1), rad | d_i, m | theta offset, rad | Variable |
|---|---:|---:|---:|---:|---|
| 1 | 0 | 0 | 0.120 | 0 | base q1 |
| 2 | 0 | pi/2 | 0 | 0 | shoulder q2 |
| 3 | 0.0565 | 0 | 0 | 0 | elbow q3 |
| 4 | 0.0615 | 0 | 0 | 0 | wrist q4 |

For each row, both implementations use:

`A_i = Rx(alpha_(i−1)) Tx(a_(i−1)) Rz(q_i + offset_i) Tz(d_i)`

The base transform is identity. The TCP/tool transform is a fixed translation
of 0.085 m along frame {4}'s x axis. FK multiplies
`base @ A1 @ A2 @ A3 @ A4 @ tool`. There is no base-minus-90 conversion.
Lengths are axis-to-axis; measured hardware dimensions replace the defaults.
The GUI's GEOMETRY entries are mm and are divided by 1000 once when building
its model. Changing a servo offset belongs in calibration, not GEOMETRY.

Frame {1} is at the top of the base post and rotates about its vertical axis.
Frame {2} has the same origin but its z axis is the shoulder axis, perpendicular
to the arm's pitch plane; its x axis follows the upper arm. Frame {3} is at the
elbow, and frame {4} at the wrist; their x axes follow the next link/tool.
The duplicate shoulder point in FK is intentional: frames {1}/{2} have different
orientations at the same origin. In this Craig convention, joint i rotates
about z_i; do not copy a standard-DH z_(i−1) geometric-Jacobian rule unchanged.

## 2. Code responsibilities and units

| File | Role / connection to MATLAB |
|---|---|
| `python/miniarm_source_model.py` | Defines the same MDH, base/tool, default seed, limits and collision configuration as `matlab/miniarm_model.m`. Its planar helpers are historical checks, not the GUI solver. |
| `python/kinematics.py` | `mdh`: A and analytic derivative D. `fk`/`position`: chain multiplication and TCP. `jacobian`: analytic partial derivative or central finite differences. `solve`: the same DLS/backtracking loop as `MiniArmIK.solve`. Internal units m/rad. |
| `python/collision.py` | Same capsule/floor geometry and continuous joint-edge checks as `MiniArmCollision.m`; Python indices are MATLAB indices minus one. |
| `miniarm_controlC/miniarm_ik.py` | GUI adapter: mm/degrees at public display helpers, m/rad in the solver. Derives model joint limits from raw calibration limits, reorders channels, rounds commands and checks actual sequential execution. |
| `miniarm_controlC/miniarm_calibration.py` | Loads/checks calibration; implements forward/inverse piecewise-linear conversion, legacy-angle conversion and raw limits. All four joints use `servo_to_q`/`q_to_servo`. |
| `miniarm_controlC/miniarm_calibrate.py` | Measurement form. New entries are DH q in degrees directly; it writes `angle_convention=joint`. Raw commands/limits are integers; measured angles may have decimals. No IK or serial I/O. |
| `miniarm_controlC/miniarm_controlC.py` | Tk GUI, serial/emulator, upright initialization, preview/Move gating, and acknowledgement-driven execution. The preview camera only changes the drawing. |
| `python/servo_export.py` | Separate, guarded CSV export example with affine calibration `raw = zero_deg + gain*q_deg`. It is not the GUI's calibration class and is not called by the GUI. |
| `python/audit_original.py` | Historical audit of the supplied old planar API. It is not an alternative production solver. |
| `python/test_*.py`, `miniarm_controlC/test_*.py` | Numerical, calibration, frame, GUI-logic, projection, startup and command-path checks. |

`python/kinematics.py` also contains an explicitly generic `example_model()`;
that model and its standalone CLI are not the miniArm model. For the miniArm
reference run `python python/miniarm_source_model.py --plot`, or use the GUI.

## 3. Jacobian and IK are the same calculation

The position Jacobian has three rows (world x/y/z) and four columns (joint q).
Its internal units are metres/radian. Column j is obtained from the FK product
by replacing only A_j with D_j = partial A_j / partial q_j, retaining every other
factor and the tool transform. The position column of the resulting derivative
matrix is `[dx/dq_j, dy/dq_j, dz/dq_j]`. The GUI helper multiplies J by 1000 to
report mm/rad. A degree input to that helper does not make its derivative mm/deg.
The finite-difference implementation perturbs by radians, as MATLAB does.

Each solve iteration evaluates `e = target - FK(q)`, then J at that same q:

`dq = J.T @ solve(J @ J.T + lambda**2 * I, e)`

Both use lambda=0.01, max joint-vector step=0.10 rad, max 400 iterations, TCP
tolerance 0.0001 m, and up to 16 backtracking scales. A trial is
`clip(q + stepScale*dq, lower, upper)`. Only a lower-error, collision-clear
candidate is committed as the next q. A stalled preview is not a valid solution.

**Same model is not a promise of identical joint answers.** MATLAB demo starts
at its bent `m.q0`; the GUI starts from the current commanded pose, which is
upright after initialization. A redundant, position-only local solver may find
a different solution or stall from a different seed. Calibrated hardware limits
can also be smaller than simulation limits. Compare FK error and constraints;
use identical seeds and limits when comparing numerical iteration histories.

## 4. Calibration is an inverse mapping, not servo = joint angle

For two neighboring measured pairs `(s_k,q_k)` and `(s_(k+1),q_(k+1))`, the
stored forward equation is:

`q(s) = q_k + (q_(k+1)-q_k) * (s-s_k)/(s_(k+1)-s_k)`

To command an IK angle, the inverse equation is:

`s(q) = s_k + (s_(k+1)-s_k) * (q-q_k)/(q_(k+1)-q_k)`

The software selects the appropriate segment (including decreasing mappings),
then rounds s to an integer. Five samples describe a piecewise-linear mapping;
the GUI does not fit one global least-squares line. With an affine calibration,
these equations reduce to the familiar offset/gain equation. Limits are checked
after conversion; the GUI does not clip an unreachable servo target into range.
Beyond sampled endpoints the nearest segment is extended and validation warns;
measure across the intended safe range instead of relying on extrapolation.

New JSON stores q directly. Older JSON without `angle_convention` retains its
original interpretation: base q=a, shoulder q=180−a, elbow/wrist q=90−a. That
conversion is applied once, inside calibration. Loading an old file into the
new form preserves its mapping; it cannot determine whether the measurements
were mistakenly entered using a different reference. Do not merely add the
new metadata to an old table without verifying/converting its angle values.

| Mathematical joint | q index | Raw command channel / index | Physical Arduino pin |
|---|---:|---|---:|
| Base | 0 | S5 / 4 | 3 |
| Shoulder / arm | 1 | S4 / 3 | 4 |
| Elbow / forearm | 2 | S3 / 2 | 5 |
| Wrist | 3 | S2 / 1 | 6 |
| Claw opening | separate | S1 / 0 | 7 |

Messages contain `[S1,S2,S3,S4,S5]`, not the q order. Only the claw is inverted
in firmware (`Servo.write(180−S1)`); Python keeps the raw S1 convention used by
calibration. Do not invert the claw again in Python. The four positioning
channels receive their raw values unchanged.

## 5. What happens after Calculate and Move

Calculate reads XYZ mm, decodes the current raw commands back to q radians,
converts XYZ to metres, and calls the shared solver. On success it converts
accepted history poses to integer commands, preserves claw opening, and checks
each actual sequential J leg: base, shoulder, elbow, wrist, claw. Checks include
piecewise-calibration knots and continuous joint interpolation. It recomputes
FK after rounding and requires predicted final miss <=10 mm. That command-rounding
tolerance is separate from the IK solver's 0.1 mm mathematical tolerance.

Only a successful, checked plan enables Move Arm. Move rechecks target/start
state and the plan, then sends one waypoint at a time, waiting for OK. The
firmware does not recalculate IK or apply DH offsets. Its P reports are command
estimates, not measured physical angles. Reloading calibration invalidates the
old preview and requires explicit initialization. Upright first enable uses
q=[90,90,0,0] and immediate INIT; later upright requests use checked gradual J.

## 6. Findings and corrections from this audit

- The MDH matrix formula, frame convention, TCP offset, Jacobian and DLS method
  agree with MATLAB source. No kinematic sign/axis correction was needed.
- Default synthetic GUI elbow/wrist limits were +/-90 degrees, whereas MATLAB
  simulation uses +/-160. They now match MATLAB exactly: base/shoulder 0..180,
  elbow/wrist -160..160. Synthetic raw mappings are base q=s, shoulder q=180−s,
  elbow/wrist q=180−2s with raw safe range 10..170. These invented mappings are
  for simulation only. They are never written to the hardware calibration.
- All four GUI joints now go through the same DH-q calibration API, including
  the base, avoiding a separate base conversion path.
- Calibration rejects fractional raw commands and limits rather than silently
  rounding saved input; physical q measurements retain decimal precision.
- The legacy reference-pose helper no longer clips an unavailable upright
  reference into a different posture.
- Emulator INIT/reset, pre-initialization motion rejection, mode validation and
  CFG timing/ranges were aligned with the firmware. No Arduino change is needed.

Default `--simulate` ignores a local calibration.json and uses the synthetic
mapping. `--simulate --use-calibration` explicitly tests the saved hardware
mapping/limits. Hardware uses the saved calibration. Your screenshot's old
table reproduces the exact 45-update, 93.2305 mm stall; its entered reference
angles cannot be repaired simply by changing solver code or ignoring limits.

## 7. Verification and limits

Run from `lab03_inverse_kinematics`:

```console
python -m unittest discover -s python -v
python -m unittest discover -s miniarm_controlC -p "test_*.py" -v
```

Audit result: **49 tests passed, 2 native Tk tests skipped without a display**.
All Python sources compile. New checks compare the MATLAB model configuration
including collision indices and GUI simulation limits; 60 random configurations
against independent elementary transforms, XYZ formulas and an analytic Jacobian;
central finite differences; signed/nonlinear calibration inversion and channel
ordering; and the real GUI Calculate/Move/poll methods through the emulator with
both direct-q and legacy calibration. Existing checks cover general transforms,
collision rejection, startup, rounding, limits and stale plans.

Reference q=[110,70,-80,-40] degrees gives TCP in mm:
`[-46.01086233, 126.41380530, 97.29949248]`.
From upright, target [100,100,50] mm converges in 42 updates with 0.0103 mm
mathematical residual. Under the new synthetic mapping its rounded final miss
is 0.8783 mm; final S1..S5 commands are [40,107,121,145,45].

MATLAB/Octave and native Tk windows were not executed in this audit environment;
MATLAB parity here means source/configuration comparison plus independent
numerical checks, not a new MATLAB runtime comparison. No hardware was moved.
Your actual calibration.json was not supplied: physical offsets, directions,
geometry and accuracy still require independent measurements/holdout poses.
Collision capsules remain approximations, especially with the camera/payload.

For a direct MATLAB check of the reference pose, run:

```matlab
m = miniarm_model();
q = deg2rad([110;70;-80;-40]);
[p,T,P] = MiniArmIK.fk(m,q);
disp(1000*p)
disp(1000*MiniArmIK.jacobian(m,q,'partial')) % mm/rad
```
