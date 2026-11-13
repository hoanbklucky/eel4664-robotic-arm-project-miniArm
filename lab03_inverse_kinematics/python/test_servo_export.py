import tempfile
from pathlib import Path
import unittest
import numpy as np
from kinematics import example_model, position, solve
from servo_export import Calibration, export_solution


class ServoExportTests(unittest.TestCase):
    def test_guards_mapping_and_limits(self):
        m = example_model(); target = position(m, m.q0)
        q, r = solve(m, target)
        # Synthetic calibration for testing only, deliberately nonsequential IDs.
        cal = [Calibration(i, 90, (-1)**j, 0, 180, True)
               for j, i in enumerate([5, 4, 3, 2])]
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temp:
            out = Path(temp)/'angles.csv'
            with self.assertRaises(ValueError): export_solution(out, m, target, q, r, cal)
            self.assertFalse(out.exists()); m.example_only = False
            rows = export_solution(out, m, target, q, r, cal)
            np.testing.assert_allclose([row[3] for row in rows],
                                       90+np.array([1,-1,1,-1])*np.rad2deg(q))
            self.assertEqual([row[1] for row in rows], [5,4,3,2])
            self.assertIn('servo_deg', out.read_text())
            with self.assertRaises(ValueError): export_solution(out, m, target, q, {'success':False}, cal)
            with self.assertRaises(ValueError): export_solution(out, m, target+.01, q, r, cal)
            with self.assertRaises(ValueError): export_solution(out, m, target, q, r, [cal[0]]*4)
            invalid = cal.copy(); invalid[0] = Calibration(5,90,1,0,180)
            with self.assertRaises(ValueError): export_solution(out, m, target, q, r, invalid)
            invalid[0] = Calibration(5,190,1,0,180,True)
            with self.assertRaises(ValueError): export_solution(out, m, target, q, r, invalid)


if __name__ == '__main__': unittest.main()
