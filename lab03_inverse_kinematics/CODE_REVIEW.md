# Review of the instructor's miniArm code

## Main finding

The supplied `miniarm_ik.py` correctly implements Craig modified-DH FK, the
**2x3 planar position Jacobian**, and a damped numerical IK update with consistent
units. There is no Jacobian sign or degree/radian conversion error in the code
reviewed. This establishes mathematical correctness for the declared model, not
its physical calibration or collision safety.

The source was read directly and executed in memory for the audit. The source
GUI, calibration code and Arduino firmware were also reviewed. The original
instructor files, answer-key file and calibration data are not republished by
this update. The new simulation uses the dimensions and conventions needed to
reproduce the mathematical model.

## Why the Jacobian is correct

Your base is handled separately. In the base-selected vertical plane:

```math
d=L_1\cos q_1+L_2\cos(q_1+q_2)+L_3\cos(q_1+q_2+q_3),
```

```math
h=H+L_1\sin q_1+L_2\sin(q_1+q_2)+L_3\sin(q_1+q_2+q_3).
```

Your Jacobian is exactly the partial derivative of these two equations with
respect to the three joint angles in **radians**, so its units are **mm/rad**.
Although the API takes degrees, it converts those angles before evaluating
sine/cosine. The inverse-kinematics update therefore comes out in radians, and
your `np.degrees(dq)` conversion before updating q is correct.

The +90 degree twist at the shoulder in your MDH table gives positive pitch
motion upward with your joint sign convention. The earlier generic simulation
used a different sign convention; the default model has now been changed to
match yours, rather than changing your correct Jacobian signs.

Your method is already **numerical IK using an analytically calculated
Jacobian**. A finite-difference Jacobian is an interchangeable approximation;
it is not a fundamentally different type of IK.

## Executed checks on your original `miniarm_ik.py`

Dimensions: H=120, L1=56.5, L2=61.5, L3=85 mm.

| Check | Result |
|---|---|
| MDH FK versus independent planar equations at 100 random poses | Maximum difference 7.66e-14 mm |
| Your Jacobian versus central differences of your FK, h=1e-5 rad | Maximum Frobenius discrepancy 8.18e-9 mm/rad |
| Reachable (d,h)=(134.5268,97.2995) mm | 5 updates; residual 0.09433 mm |
| Reachable (d,h)=(123.1803,62.3734) mm | 9 updates; residual 0.06711 mm |
| Reachable (d,h)=(121.1154,154.7003) mm | 10 updates; residual 0.06539 mm |
| Unreachable (d,h)=(1000,120) mm | Returned None |

All three IK tests started at (60,-60,-60) degrees. These goals were generated
from known model poses. They demonstrate convergence for these cases, not all
targets or seeds. Run `audit_original.py` against your local file to reproduce.

## Planar target versus XYZ target

Your 2x3 Jacobian is sufficient when the user specifies base angle, distance and
height separately. With b = base_angle - 90 degrees, XYZ is:

```math
x=d\cos b,\qquad y=d\sin b,\qquad z=h.
```

To convert an ordinary XYZ target with nonnegative radial distance, set
`d=hypot(x,y)`, `base_angle=90+degrees(atan2(y,x))`, and `height=z`, then check
the base's calibrated range. At x=y=0, keep a chosen/current base heading; yaw
is not determined by XYZ. Signed negative distance can represent a different
configuration branch and must be handled deliberately.

The supplied simulation also offers full XYZ IK using a 3x4 Jacobian. This
does not mean your existing 2x3 formulation was missing a row: it solves a
different task. In both formulations the gripper pitch remains free. Pick/place
may need a pitch constraint if the gripper must approach vertically.

## Findings outside the Jacobian

| Finding | Consequence | Recommended change before hardware use |
|---|---|---|
| Original IK ignores joint limits, then GUI clamps servo targets | Tiny solver error can become a large final miss | Derive joint limits from calibrated servo ranges and enforce them inside IK; always recompute FK after rounding |
| GUI `MAX_MISS_MM=25` but lab's stated reach criterion is 10 mm | A move can be accepted despite failing the lab criterion | Select and enforce one explicit post-rounding error threshold |
| GUI `check_pose()` checks only the candidate final q; Arduino `J` moves joints sequentially | Intermediate poses can intersect the table/base despite a valid endpoint | Check the actual base/arm/forearm/wrist command path; endpoint checks are not collision guarantees |
| `P` messages contain software ramp-command values | The GUI's current_q is estimated from commands, not measured encoder feedback | Label it as a commanded-position estimate; validate physical settling and sag |
| GUI float parsing and calibration checks do not explicitly reject all NaN/Inf cases | Malformed input can raise exceptions or propagate invalid calculations | Require finite numbers at input, calibration, solver output and before rounding |
| IK progress check compares every 20 iterations against 95% of the prior checkpoint | A reachable target can be abandoned during slow progress | Report stalled separately and consider residual-based backtracking or a more deliberate stagnation check |
| Calibration extrapolates outside measured points | Angles outside calibration coverage are uncertain | Prefer measured coverage throughout the permitted range; do not treat extrapolated angles as measured |

The GUI's STOP handler switches out of knob mode before STOP, which addresses
continued knob control in that GUI flow. The firmware's standalone STOP command
only freezes its targets; if it remains in K mode, later knob updates resume.
It is not a latched emergency-stop mechanism. Startup INIT also commands a pose
immediately rather than ramping from a measured physical pose.

These findings do not invalidate the Jacobian. They identify the remaining
interface, calibration and motion-execution work. No hardware motion was run,
and no firmware compilation or MATLAB execution is claimed.

## What changed in this repository

- Default MATLAB model now uses your instructor dimensions, +90 degree twist,
  and the base-angle-to-yaw conversion.
- `verify_miniarm` checks reference poses, your planar derivative formula, and
  exact versus finite-difference Jacobians.
- `miniarm_source_model.py --plot` animates the same model and prints both GUI
  angles and XYZ units. Its joint limits are explicitly simulation limits.
- `audit_original.py` checks your existing module without changing its API.
- Original generic geometry remains isolated in regression fixtures.

The new full-XYZ solver enforces the configured model limits and decreasing
position error. It does not by itself resolve physical collision checks,
piecewise calibration integration or safe servo execution in the original GUI.
