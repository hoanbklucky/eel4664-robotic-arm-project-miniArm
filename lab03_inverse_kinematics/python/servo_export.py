"""Export calibrated servo targets for inspection; no hardware I/O."""
from dataclasses import dataclass
import csv
import numpy as np
from kinematics import position


@dataclass(frozen=True)
class Calibration:
    servo_id: int
    zero_deg: float
    gain: float
    minimum_deg: float
    maximum_deg: float
    verified: bool = False


def export_solution(path, model, target, q, result, calibration):
    from collision import clearance
    model.validate()
    if model.example_only:
        raise ValueError('Replace example geometry/limits before servo export')
    q = model.check_q(q, limits=True)
    if clearance(model,q)[0]<=0:
        raise ValueError('Solution violates configured collision clearance')
    target = np.asarray(target, dtype=float)
    if (target.shape != (3,) or not np.isfinite(target).all()
            or not result.get('success', False)
            or np.linalg.norm(target-position(model, q)) > 1e-4):
        raise ValueError('Require a successful solution for this target')
    if len(calibration) != len(q):
        raise ValueError('One calibration per mathematical joint required')
    rows = []; ids = set()
    for i, (qi, c) in enumerate(zip(q, calibration), start=1):
        values = [c.zero_deg, c.gain, c.minimum_deg, c.maximum_deg]
        if (not c.verified or not np.isfinite(values).all() or c.gain == 0
                or c.minimum_deg >= c.maximum_deg or not isinstance(c.servo_id, int)
                or c.servo_id < 0 or c.servo_id in ids):
            raise ValueError('Invalid/unverified calibration or duplicate servo ID')
        ids.add(c.servo_id)
        angle = c.zero_deg + c.gain*np.rad2deg(qi)
        if not c.minimum_deg <= angle <= c.maximum_deg:
            raise ValueError('Servo angle outside calibrated range; do not clip')
        rows.append([i, c.servo_id, float(qi), float(angle)])
    # Validate every joint before opening the output file.
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f); writer.writerow(['joint', 'servo_id', 'q_rad', 'servo_deg'])
        writer.writerows(rows)
    return rows
