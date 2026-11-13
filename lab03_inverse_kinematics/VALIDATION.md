# Validation record

## Executed for this revision

NumPy reference, Python 3.12 on Windows:

```text
python -m unittest discover -s python -v
Ran 16 tests ... OK
```

Coverage:

- 100 random example-chain poses: FK and exact Jacobian against independently
  written closed-form yaw/planar equations; central differences against that
  Jacobian; small-displacement predictions.
- General MDH twists/displacements, nonzero theta offsets, nonidentity base and
  tool transforms: exact versus finite-difference Jacobian.
- Three known-reachable goals with both Jacobian methods: convergence to
  <=0.1 mm, all iterates within limits, nonincreasing residual.
- An unreachable target, a zero-error initial pose, a singular seed (finite
  numerical output), malformed inputs and invalid transforms.
- Servo conversion: signed gains, nonsequential IDs, calibrated limits,
  example-model guard, failed-solve and stale-target rejection, unverified and
  duplicate calibration rejection. Synthetic values are used only in tests.

Default NumPy demonstration:

```text
target (m): [0.20720755 0.07563666 0.08087362]
q (deg): [20.05352801 -45.27806251 70.08946821 -16.9410312]
converged residual (m): 9.866220734062581e-06
```

The final angles differ from the pose used to generate the target because
position-only IK of this four-joint chain is redundant.

## Provided but not executed in this environment

The source-model command-line demonstration also converged to residual
0.006306 mm for XYZ [-46.01086233,126.4138053,97.29949248] mm in the current
x-right/y-forward frame. GUI animation was
not rendered here because matplotlib is not installed in this Python runtime;
install the provided requirements to run `--plot` locally.

MATLAB and Robotics System Toolbox were unavailable. `verify_ik.m` and
`verify_toolbox.m` are provided for local execution; no MATLAB runtime or
toolbox pass is claimed. The MATLAB implementation follows the same equations
as the tested NumPy reference, but Python results do not establish that MATLAB
scripts executed successfully. Run both verification functions in MATLAB before
classroom use. `verify_ik` always uses the fixed illustrative fixture, so it
remains useful after you edit `miniarm_model`.

## Not established by these checks

The instructor's supplied `miniarm_ik.py` has now been executed in memory and
audited with `python/audit_original.py`; see CODE_REVIEW.md for numerical results.
Three additional regression tests exercise the instructor geometry and compare
its planar Jacobian formula with the generic MDH implementation. Hardware FK
accuracy, actual joint/servo limits, servo IDs,
pin mapping, serial firmware, contact/collision behavior, motor dynamics and
physical reaching accuracy have not been tested. No Webots world is included.

## Collision revision

See COLLISIONS.md for the reproduced old-solver base intersection and the new
capsule/path checks. Six additional Python tests cover capsule segment distance,
colliding initial poses, old-versus-new base intersection, an unsafe interpolation
between clear endpoints, safe goal paths and collision configuration validation.
The source-model default still converges with collision checks enabled. MATLAB
collision regressions are provided in `verify_collision.m` but were not run here.
Provisional radii must be measured before interpreting this as a physical model.

## Latest instructor model and frame convention

The supplied `miniarm_model.m` numerics are preserved: first-row angular offset
zero, base limits [0,180] degrees, q0=[90,60,-60,-60] degrees, and
demoGoal=[110,70,-80,-40] degrees. Its leftover base_angle-90 comment was corrected.
The Python model and miniArm/collision regressions now use the same convention.
A cross-language configuration test reads the MATLAB table, limits and poses
and compares them with the NumPy reference, to catch inconsistent updates.
The original Python audit remains a historical test of the uploaded code.
