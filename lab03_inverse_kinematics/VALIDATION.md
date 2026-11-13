# Validation record

## Executed for this revision

NumPy reference, Python 3.12 on Windows:

```text
python -m unittest discover -s python -v
Ran 6 tests ... OK
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

MATLAB and Robotics System Toolbox were unavailable. `verify_ik.m` and
`verify_toolbox.m` are provided for local execution; no MATLAB runtime or
toolbox pass is claimed. The MATLAB implementation follows the same equations
as the tested NumPy reference, but Python results do not establish that MATLAB
scripts executed successfully. Run both verification functions in MATLAB before
classroom use. `verify_ik` always uses the fixed illustrative fixture, so it
remains useful after you edit `miniarm_model`.

## Not established by these checks

The instructor's current IK implementation could not be audited because its
source/attachment was not available. The example geometry is not measured
miniArm geometry. Hardware FK accuracy, actual joint/servo limits, servo IDs,
pin mapping, serial firmware, contact/collision behavior, motor dynamics and
physical reaching accuracy have not been tested. No Webots world is included.
