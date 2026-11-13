# Python GUI integration validation

## Predictable upright and workflow feedback (latest)

Post-initialization upright now preserves APP or IK mode and the claw command.
It does not send MODE A. In KNOB mode the action is disabled and the hint asks
for APP/IK first, avoiding live knob control undoing the posture. Raw manual
commands and XYZ targets are grouped; persistent next-action/result lines explain
mode restrictions, initialization, motion, successful preview and solver/path/input
failures. Enter calculates only. StringVar write tracking invalidates a preview
only when XYZ text changes, avoiding the old navigation-key invalidation behavior.

The calibration window tracks actual edits, shows an inline summary and a
selectable file path, and protects unsaved measurements on close/reload. Failed
validation or disk writes do not discard entries. Resizing expands diagnostics.

50 noninteractive GUI/adapter tests passed; 2 native Tk tests were skipped without
a display. Six new checks cover a real checked emulator return to upright from a
bent pose while retaining IK/claw, APP preservation and KNOB restrictions, visible
success/failure/input feedback, saved DH metadata and failed writes, close cancel,
and cancelled reload. Existing initialization/STOP/preview/path tests still pass.
Python sources compile. Native window layout and physical movement were not
verified in this environment; no Arduino firmware change is required.

## Active mode, hardware startup lock and acknowledged STOP (latest)

APP/KNOB/IK buttons now use their green/orange/blue colors and a sunken/bold
selected style, with a text mode/moving/stopped status. Hardware controls stay
disabled until startup OK. STOP and Reload calibration remain available. Controls
are also locked during motion or while STOP is awaiting acknowledgement and the
following position report. Arduino MODE replies update the displayed mode.

STOP cancels the Python queue, sends the stop before redrawing, and preserves
servo power. It exits KNOB mode first and suppresses automatic knob takeover
after a user stop. Intentional knob takeover during IK waits for stop confirmation
before entering KNOB mode. No Arduino firmware change was needed.

44 noninteractive GUI/adapter tests passed; 2 native-window tests were skipped.
Six new checks cover hardware locks until initialization OK, mode styling and
mode-specific controls, queue cancellation and emulator freeze, STOPPED plus
position-report gating, reset relocking, knob-mode exit and deliberate takeover.
Source compiles. Native window appearance and physical stop latency were not
measured; no physical hardware was commanded. This is a software command stop,
not a power-off or guaranteed instantaneous emergency stop.

## Direct IK preview on simulation startup (latest)

Only the emulator handles READY by preparing its upright reference automatically,
then selecting IK mode after the initialization acknowledgement. The constructor
processes the emulator READY before returning. Users can enter XYZ and Calculate
without pressing Straighten arm upright. The button remains available for later
checked return moves. Valid saved-calibration reload in the emulator also restores
this ready state. A physical serial link remains disabled and requires explicit
initialization; neither READY nor reload automatically enables physical servos.

38 noninteractive GUI/adapter tests passed; 2 native Tk tests were skipped without
a display. Three new checks cover direct Calculate after emulator READY, reset,
and zero initialization commands on hardware READY. The actual Calculate/Move
pipeline tests still pass for both calibration formats. Python source compiles.
No physical hardware or native Tk window was run in this revision.

## Corrected +/-90-degree limits and upright calibration check (latest)

The instructor confirmed +/-90 degrees for elbow/wrist. MATLAB and Python model
limits and the default synthetic mapping now match that range. Calibration can
narrow nominal limits but cannot enlarge them. The prior +/-160 correction was
incorrect for this miniArm. Upright now logs the per-joint q-to-raw conversions.
51 tests pass, with 2 native Tk tests skipped without a display. New tests cover
measured offsets and opposite servo directions for both direct-q and legacy
formats, and prevent saved calibration from expanding the +/-90 limits.
Synthetic upright commands decode to q=[90,90,0,0], while an invented offset
calibration yields upright raw [70,60,120,120,100] and decodes to the same q.
These are software conversion tests, not measurements of this user's robot.
No hardware movement or new MATLAB runtime execution was performed.

The [pipeline audit](PYTHON_MATLAB_AUDIT.md) and calibration guide contain current
values. Sections below record earlier revisions. Current target [100,100,50]
converges at 0.0103 mm, with rounded miss 0.8783 mm under the +/-90 synthetic map.

## MATLAB/Python consistency audit (latest)

See [the complete pipeline audit](PYTHON_MATLAB_AUDIT.md). All 49 noninteractive
tests passed; 2 live Tk tests were skipped without a display. The new tests
compare the GUI's complete default model, including joint limits and collision
indices, with MATLAB source, independently check MDH/FK/Jacobian and test the
actual GUI Calculate/Move/poll code through the emulator for both calibration
formats. Sources compile. No MATLAB engine, native Tk window or hardware run
was performed here.

The default synthetic mapping now covers MATLAB's +/-160-degree elbow/wrist
limits instead of +/-90. For target [100,100,50] mm from upright the solver still
converges at 0.0103 mm; changed synthetic command resolution makes the rounded
miss 0.8783 mm. These figures supersede the prior mapping's figures below.
All four joints use the same DH-q calibration API. Fractional raw inputs are
rejected, while decimal measured joint angles remain supported. Emulator INIT,
pre-init guards, mode validation and CFG timing/ranges now match firmware.

## Reproducible simulation calibration

Default --simulate now uses the synthetic calibration regardless of whether a
local calibration.json exists. --simulate --use-calibration explicitly loads
the saved hardware mapping and fails on invalid/missing data. Hardware still
uses measured calibration. Synthetic mode cannot silently switch mappings via
Reload calibration. No saved calibration file is rewritten by this change.

Five new noninteractive tests passed. The instructor screenshot was reproduced
from its visible older table: elbow/wrist limits become [0,180] degrees, and
[100,100,50] mm from upright stalls after 45 updates at 93.2305 mm error. With
default synthetic calibration, the same target converges in 42 updates with
0.0103 mm residual; the rounded plan has 42 checked waypoints and 0.4437 mm
predicted final miss. Every sequential leg was checked again in the regression.
The other 22 noninteractive GUI/adapter tests passed (27 total in this revision);
one native Tk preview test was skipped without a display. Source compiles.
No native window or physical hardware execution is claimed.

The screenshot failure comes from interpreting entered angles under the older
calibration convention. This change isolates offline exercises; it does not
repair a hardware calibration with mixed angle references. Preserve the measured
file and verify its entries using CALIBRATION_GUIDE.md before hardware use.

## Fixed upright initialization

The calibrated startup target is q=[90,90,0,0] degrees, independent of sliders
and the model's IK seed. The button is Straighten arm upright. An unavailable
upright reference is rejected rather than clipped. First enable sends INIT;
subsequent initialized clicks validate the sequential path, send a gradual J
move, and preserve the current claw command. Without calibration, the separate
label Enable raw neutral (uncalibrated) identifies the raw-90 bootstrap action.

Four headless startup tests passed, covering fixed reference selection,
calibrated-limit rejection with no commands sent, subsequent checked motion,
blocked-path rejection, claw preservation and uncalibrated labeling. The 18
noninteractive integration/calibration/preview tests also passed; the live Tk
preview test was skipped without a display. Python source compiles. Physical
initialization and native Tk interaction were not tested for this revision.

## Calibration explanation and direct joint-angle entry

The calibration form now accepts DH joint q in degrees directly, including
negative elbow/wrist angles and 0 degrees for aligned links. Its help explains
why raw commands differ from joint angles, how to measure a pair, and why XYZ
targets still require joint-angle calibration. New JSON files declare
`angle_convention=joint`; files without metadata retain the legacy convention.
Loading a legacy file into the form converts its table for display and saving.

Checks executed for this revision:

- **34 noninteractive tests passed**: 16 numerical, 6 integration, 6 preview
  projection/Canvas and 6 calibration tests.
- Calibration tests compare legacy and converted mappings over every integer
  raw command 0..180 for all four joints, round trips, raw limits, startup
  commands and model limits. They also cover signed-angle form data, negative
  plot labels, the worked wrist example and rejection of nonfinite inputs.
- Python source compiles. Full native Tk interaction and physical measurement
  accuracy were not tested here; the optional preview Tk test was skipped
  because no display connection is available. No hardware was commanded.

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
python -m unittest discover -s . -p test_calibration.py -v
python miniarm_controlC.py --simulate
```

