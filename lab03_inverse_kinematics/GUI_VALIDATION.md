# Python GUI integration validation

## Checks executed

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
python miniarm_controlC.py --simulate
```
