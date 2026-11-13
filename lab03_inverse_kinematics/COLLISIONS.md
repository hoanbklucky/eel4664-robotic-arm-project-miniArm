# Reaching XYZ is not sufficient: collision handling

The original solver minimized XYZ error within joint limits. It could therefore
return an accurate target position with the tool passing through the base or
another link. A free claw pitch makes different postures possible, but fixing
pitch is not a collision-avoidance method; even a downward claw can collide.

Keep the 3x4 position Jacobian and free pitch. Require positive geometric
clearance in addition to small XYZ error and legal joint angles.

## Update MATLAB Online

Download the latest repository ZIP and replace **all `.m` files** in your online
lab folder, including the new `MiniArmCollision.m` and `verify_collision.m`.
Then run:

```matlab
clear classes
verify_miniarm
verify_collision
demo_ik
% Then rerun your chosen XYZ target, in METRES:
% demo_ik([x; y; z])
```

Claw orientation remains unconstrained. MATLAB execution of this revision has
not been verified in the development environment; equivalent Python regressions
have been executed.

## Geometry that is actually checked

The default `miniarm_model.m` / `source_model()` approximate each physical
segment by a **capsule** (a line segment thickened by a radius, including rounded
ends). Configured pairs are base/forearm, base/tool and upper-arm/tool. Immediate
neighbours are excluded because their bodies meet at a joint. The non-base
capsules are also checked against the horizontal table plane.

| Item | Provisional simulation value |
|---|---:|
| Base capsule radius | 20 mm |
| Upper-arm radius | 6 mm |
| Forearm radius | 6 mm |
| Tool/gripper radius | 6 mm |
| Additional clearance margin | 3 mm |
| Table plane | world z=0 |

**These radii are not manufacturer measurements.** Measure the actual housing,
gripper, camera, brackets and base envelope before hardware use. In particular,
a thin tool capsule may underrepresent the open gripper or front-mounted camera.
The DH table specifies axes and lengths; it does not specify occupied volume.
Connected-link contacts, cables, torque, backlash and additional objects are not
fully modeled. A differently structured robot needs different segment indices
and collision pairs, even if its DH table is valid.

In MATLAB, FK columns are table origin, shoulder, shoulder, elbow, wrist and TCP.
`m.collision.edges=[1 2;3 4;4 5;5 6]` defines the four capsules from those points.
The repeated shoulder point comes from the coincident base/shoulder DH origins.

For a pair i,j:

```math
c_{ij}=\operatorname{distance}(segment_i,segment_j)-r_i-r_j-margin.
```

Positive clearance is required. Table clearance is the lowest endpoint z minus
the capsule radius, table height and margin. The minimum over the configured
checks is returned as `r.clearance` in metres, with `r.contact` identifying the
limiting pair. Touching at zero clearance is rejected.

## What changes inside IK

1. Reject a seed already in collision (`seed_in_collision`), even if FK exactly
   matches the target.
2. Compute the usual damped position-Jacobian step.
3. Backtrack candidate steps. Accept only a decreasing position error AND a
   clear interpolated joint segment from the current q to the candidate q.
4. Require positive final clearance before declaring success.

The interpolation check uses a conservative motion bound: point travel is at
most total kinematic chain length times the sum of absolute joint changes.
Clearance between two moving capsules changes by at most twice that bound.
An interval is accepted only when the bound establishes clearance; otherwise
it is subdivided. At depth 12 it fails closed if still unresolved. This checks
the continuous straight joint interpolation for the **configured approximation**,
rather than assuming that clear endpoint poses imply a clear path.

The search can stop near a collision boundary (`stalled_or_collision`), even
when another posture or route exists. It is not a global collision-aware motion
planner and does not guarantee that a different seed will solve the problem.
No collision-free solution found is different from a proof of unreachability.

The accepted optimization history is checked segment by segment. Sending only
the final q, or moving the servos one at a time, creates a different path that
must be checked separately. The original GUI/firmware are not automatically
made collision-aware by these simulation changes. Servo export checks final
clearance but does not command or certify a hardware execution path.

## Executed regression example

With the instructor geometry, target XYZ approximately
`[0,14.7810,106.2560]` mm in the current x-right/y-forward frame gives a useful
negative test (base angle 90 degrees). The previous solver
converges with a model position error of 0.03094 mm while its tool/base clearance
is **-14.22 mm** under the provisional capsule model. The revised solver rejects
that target result and returns failure with a clear final iterate.

This target is inside the configured expanded base envelope, so it is not
expected to succeed. It illustrates why a very small FK residual is insufficient.
It is a reproduced regression, not a claim to know the exact XYZ or q used in
the user's screenshot.

Additional tests check intersecting/parallel/skew/degenerate/nearly parallel
segments, a colliding seed with zero XYZ error, a collision between two clear
interpolation endpoints, and successful clear paths to the default target.
