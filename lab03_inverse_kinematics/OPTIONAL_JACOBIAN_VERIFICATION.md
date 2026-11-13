# Optional Jacobian verification and solver details

This page is enrichment material for students who want to inspect the mathematics
or software checks more closely. **None of these calculations, Python checks or
unit tests is required for the main lab report**, unless the instructor assigns
them separately. Return to the [student guide](STUDENT_GUIDE.md).

## Verify Jacobian columns numerically

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


## Inspect one IK iteration

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


For a longer derivation, see the [matrix differentiation walkthrough](matlab/JACOBIAN_WALKTHROUGH.md).

