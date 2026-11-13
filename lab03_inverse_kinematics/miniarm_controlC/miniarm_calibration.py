"""
miniarm_calibration.py  -  reading, checking and using calibration.json
=======================================================================

Shared by miniarm_calibrate.py (which writes the file) and miniarm_controlC.py
(which reads it), so both convert between servo values and angles the same way.

What a calibration is
---------------------
For each joint, a few (servo value, measured angle) pairs. Angles are measured
RELATIVE TO THE PREVIOUS LINK, with 90 = in line with it:
    base    : relative to straight ahead   (90 = straight ahead, larger = to the arm's left)
    arm     : relative to the base post    (90 = straight up,    larger = bent forward)
    forearm : relative to the arm link     (90 = in line,        larger = bent forward)
    wrist   : relative to the forearm      (90 = in line,        larger = bent forward)
Between points the angle is a straight line (piecewise linear), so the curve passes
through every measured point.

Each joint also has a safe range (min, max servo value), and the claw has open and
closed values.

calibration.json looks like:
{
  "joints": {
    "base":    {"points": [[22, 0], [67, 45], [112, 90], [143, 120], [174, 150]], "min": 0, "max": 174},
    "arm":     {...}, "forearm": {...}, "wrist": {...}
  },
  "claw": {"open": 40, "closed": 94, "min": 40, "max": 94}
}
"""

import json

JOINTS = ("base", "arm", "forearm", "wrist")
SERVO_INDEX = {"claw": 0, "wrist": 1, "forearm": 2, "arm": 3, "base": 4}   # read_mini.ino order
IK_JOINTS = ("arm", "forearm", "wrist")                                   # q1, q2, q3 in miniarm_ik.py

MIN_POINTS = 2
RECOMMENDED_POINTS = 5
KINK_RATIO = 2.0      # warn when one segment is this many times steeper or flatter than its neighbor


class CalibrationError(Exception):
    """The calibration file is missing, unreadable, or has a problem that must be fixed."""


class Calibration:
    def __init__(self, data):
        self.data = data
        self.points = {}
        for name in JOINTS:
            pts = sorted((float(s), float(a)) for s, a in data["joints"][name]["points"])
            self.points[name] = pts
        claw = data["claw"]
        self.claw_open = int(claw["open"])
        self.claw_closed = int(claw["closed"])

    # ------------------------------------------------------------------ files
    @classmethod
    def load(cls, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except FileNotFoundError:
            raise CalibrationError(f"{path} not found. Run miniarm_calibrate.py first.")
        except (OSError, json.JSONDecodeError) as e:
            raise CalibrationError(f"Could not read {path}: {e}")
        errors, _ = check(data)
        if errors:
            raise CalibrationError("Problems in the calibration file: " + "; ".join(errors))
        return cls(data)

    @staticmethod
    def save(data, path):
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)

    # ------------------------------------------------------------------ servo <-> measured angle
    def servo_to_angle(self, name, servo):
        """Servo value -> measured angle (deg), through the joint's points.
        Beyond the first or last point, the nearest segment is extended."""
        return _interpolate(self.points[name], servo)

    def angle_to_servo(self, name, angle):
        """Measured angle (deg) -> servo value (float, not rounded)."""
        by_angle = sorted((a, s) for s, a in self.points[name])
        return _interpolate(by_angle, angle)

    def measured_span(self, name):
        """Smallest and largest measured angle for a joint."""
        angles = [a for _, a in self.points[name]]
        return min(angles), max(angles)

    # ------------------------------------------------------------------ safe ranges
    def limits(self, index):
        """(min, max) servo value for a servo index 0-4 (read_mini.ino order)."""
        if index == SERVO_INDEX["claw"]:
            c = self.data["claw"]
        else:
            name = next(n for n, i in SERVO_INDEX.items() if i == index)
            c = self.data["joints"][name]
        return int(c["min"]), int(c["max"])

    def all_limits(self):
        return [self.limits(i) for i in range(5)]

    # ------------------------------------------------------------------ poses
    def middle_pose(self, claw=None):
        """Every joint at 90 degrees (arm up, forearm and wrist in line, base ahead).
        Servo values for S1..S5; the claw is open unless given."""
        pose = [0] * 5
        pose[SERVO_INDEX["claw"]] = self.claw_open if claw is None else claw
        for name in JOINTS:
            i = SERVO_INDEX[name]
            lo, hi = self.limits(i)
            pose[i] = min(max(round(self.angle_to_servo(name, 90.0)), lo), hi)
        return pose

    # ------------------------------------------------------------------ the IK's angles
    # miniarm_ik.py uses q1 = arm from horizontal (90 = up, forward = smaller),
    # q2, q3 = relative to the previous link (0 = in line, forward = negative).
    # Measured angles use 90 = in line and forward = larger, so:
    #     q1 = 180 - arm      q2 = 90 - forearm      q3 = 90 - wrist
    @staticmethod
    def measured_to_q(name, angle):
        return 180.0 - angle if name == "arm" else 90.0 - angle

    @staticmethod
    def q_to_measured(name, q):
        return 180.0 - q if name == "arm" else 90.0 - q

    def q_to_servo(self, name, q):
        return round(self.angle_to_servo(name, self.q_to_measured(name, q)))

    def servo_to_q(self, name, servo):
        return self.measured_to_q(name, self.servo_to_angle(name, servo))

    def base_angle_to_servo(self, angle):
        """Base angle (90 = straight ahead, larger = left) -> servo value."""
        return round(self.angle_to_servo("base", angle))


def _interpolate(pairs, x):
    """Piecewise-linear y(x) through (x, y) pairs sorted by x; extends the end segments."""
    if len(pairs) == 1:
        return pairs[0][1]
    if x <= pairs[0][0]:
        k = 0                                   # before the first point: extend the first segment
    elif x >= pairs[-1][0]:
        k = len(pairs) - 2                      # after the last point: extend the last segment
    else:
        k = next(i for i in range(len(pairs) - 1) if pairs[i][0] <= x <= pairs[i + 1][0])
    (x0, y0), (x1, y1) = pairs[k], pairs[k + 1]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


# ----------------------------------------------------------------------------
# Checking a calibration (used by both programs)
# ----------------------------------------------------------------------------
def check(data):
    """Return (errors, warnings). Errors must be fixed before the file can be used."""
    errors, warnings = [], []
    joints = data.get("joints", {}) if isinstance(data, dict) else {}
    for name in JOINTS:
        j = joints.get(name)
        if not j:
            errors.append(f"{name}: missing")
            continue
        try:
            pts = sorted((float(s), float(a)) for s, a in j.get("points", []))
            lo, hi = float(j["min"]), float(j["max"])
        except (TypeError, ValueError, KeyError):
            errors.append(f"{name}: points and min/max must be numbers")
            continue

        if len(pts) < MIN_POINTS:
            errors.append(f"{name}: needs at least {MIN_POINTS} points (has {len(pts)})")
            continue
        if len(pts) < RECOMMENDED_POINTS:
            warnings.append(f"{name}: {len(pts)} points; {RECOMMENDED_POINTS} spread across the range is more accurate")
        servos = [s for s, _ in pts]
        if any(not 0 <= s <= 180 for s in servos):
            errors.append(f"{name}: servo values must be 0-180")
        if len(set(servos)) != len(servos):
            errors.append(f"{name}: the same servo value appears twice")
            continue
        if not (0 <= lo < hi <= 180):
            errors.append(f"{name}: safe range must satisfy 0 <= min < max <= 180")

        # Angles must keep going the same way as the servo value rises
        steps = [pts[k + 1][1] - pts[k][1] for k in range(len(pts) - 1)]
        if any(d == 0 for d in steps):
            errors.append(f"{name}: two points have the same angle")
            continue
        if not (all(d > 0 for d in steps) or all(d < 0 for d in steps)):
            errors.append(f"{name}: angles go up and then down; check the points")
            continue

        # Flag a point where the slope changes sharply (often a measuring slip)
        slopes = [(pts[k + 1][1] - pts[k][1]) / (pts[k + 1][0] - pts[k][0]) for k in range(len(pts) - 1)]
        for k in range(len(slopes) - 1):
            ratio = abs(slopes[k + 1] / slopes[k])
            if ratio > KINK_RATIO or ratio < 1 / KINK_RATIO:
                s, a = pts[k + 1]
                warnings.append(f"{name}: sharp bend at servo {s:g} / {a:g} deg; re-measure that point")

        angles = [a for _, a in pts]
        if not min(angles) <= 90 <= max(angles):
            warnings.append(f"{name}: no points on both sides of 90 deg; the middle pose is extrapolated")
        if lo < servos[0] or hi > servos[-1]:
            warnings.append(f"{name}: safe range goes beyond the measured points; angles out there are a guess (extended line)")

    claw = data.get("claw") if isinstance(data, dict) else None
    if not claw:
        errors.append("claw: missing")
    else:
        try:
            o, c, lo, hi = (float(claw[k]) for k in ("open", "closed", "min", "max"))
            if not (0 <= lo < hi <= 180):
                errors.append("claw: safe range must satisfy 0 <= min < max <= 180")
            elif not (lo <= o <= hi and lo <= c <= hi):
                errors.append("claw: open and closed must be inside its safe range")
            if o == c:
                errors.append("claw: open and closed are the same value")
        except (TypeError, ValueError, KeyError):
            errors.append("claw: open, closed, min and max must be numbers")
    return errors, warnings
