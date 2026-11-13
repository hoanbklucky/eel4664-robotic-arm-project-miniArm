# Python GUI integration validation

## Current preview projection correction

The prior hard-coded screen projection had camera-normal Z below zero and
different horizontal/vertical scale factors. It could make the world axes look
inconsistent with the instructor's above-table frame illustration. The preview
now uses an explicit orthonormal camera basis with uniform scale and an
above-table rear-right default (camera direction proportional to [1,-1,0.8]).
Camera choices are rear-right, rear, top and right side. They change only the
drawing; the FK/IK world frame, calibration and servo commands are unchanged.
The current estimate is gray/dashed when a separate purple candidate is shown.
The readout is explicitly labeled as current, and axis {0}/table are identified.

Checks executed for this revision:

- **16 numerical tests passed**.
- **6 noninteractive integration tests passed**, including checked servo paths.
- **6 projection/Canvas-renderer tests passed**: orthonormal/right-handed camera
  basis, above-table default, isotropic scaling, known direction checks, arm/TCP
  and target drawn through the same projection, all four camera views, and a
  small-window guard that prevents a negative-scale/inverted drawing on resize.
- The recorded production Canvas commands were rendered and visually inspected.
  The reference PNG is this headless rendering, not a native Tk window screenshot.
- The new interactive camera-switch Tk test is provided but was **skipped** here
  because a display connection is unavailable. The original live Tk integration
  test was not rerun successfully in this environment. No full native-window
  execution pass is claimed for this revision.
- Python source compiles. No hardware was connected or commanded.

## Earlier integration checks

- Existing numerical suite: **16 tests passed**.
- New GUI/integration suite: **7 tests passed**.
- Python files compile successfully.
- Offline Tk GUI initializes explicitly, previews without moving, executes a
  checked waypoint queue through the emulator, and cancels queued motion on STOP.
- Independent XYZ formulas and finite differences agree with the partial
  Jacobian. Base 0/90/180 degrees maps to right/forward/left.
- Tests cover calibrated conversion, rounding, limit rejection, unreachable
  targets, stale starting states, and collision rejection on sequential motion.

The shared `python/kinematics.py`, `python/collision.py` and default model are
the same core used by the existing reference checks against the MATLAB model.
No separate MATLAB engine or toolbox is needed by the GUI.

## Limits of these checks

The current Arduino sketch has not been compiled or uploaded in this environment.
No physical movement, calibration accuracy, servo timing or load behavior has
been verified. The emulator tests command handling, not mechanics. MATLAB was
not rerun as part of this GUI change; the instructor previously ran the MATLAB
simulation successfully. Collision capsules are provisional and must be reviewed
against the actual arm and attachments. A valid IK result can still be rejected
if its rounded sequential execution path fails the configured clearance check.

The six-page report template was checked by text extraction. Local document
rendering was unavailable, so its final visual layout still needs review in Word.

## Reproduce

From `miniarm_controlC`:

```console
python -m pip install -r requirements.txt
python -m unittest discover -s ../python -v
python -m unittest discover -s . -p test_integration.py -v
python -m unittest discover -s . -p test_preview.py -v
python miniarm_controlC.py --simulate
```

