"""Reproduce saved-convention failure and protect reproducible offline mode."""
import unittest
from unittest.mock import patch
import numpy as np
import miniarm_calibration as mc
import miniarm_controlC as gui
import miniarm_ik as ik


def screenshot_calibration():
    raw=[0,45,90,135,180]
    # Table visible in the instructor's older calibrator screenshot.
    angles={'base':raw,'arm':[180,135,90,45,0],
            'forearm':[90,45,0,-45,-90],'wrist':[-90,-45,0,45,90]}
    return mc.Calibration(dict(joints={n:dict(points=list(zip(raw,a)),min=0,max=180)
                                      for n,a in angles.items()},
                               claw=dict(open=70,closed=80,min=65,max=85)))


class SimulationCalibrationTests(unittest.TestCase):
    def test_default_simulation_does_not_read_saved_file(self):
        with patch.object(mc.Calibration,'load',return_value=screenshot_calibration()) as load:
            cal,synthetic=gui.load_startup_calibration(True)
        load.assert_not_called();self.assertTrue(synthetic)
        self.assertEqual(cal.claw_open,40)
        self.assertLess(cal.servo_to_q('forearm',180),0)

    def test_explicit_saved_simulation_uses_file_and_does_not_fallback(self):
        saved=screenshot_calibration()
        for simulate in (True,False):
            with patch.object(mc.Calibration,'load',return_value=saved):
                cal,synthetic=gui.load_startup_calibration(simulate,True)
            self.assertIs(cal,saved);self.assertFalse(synthetic)
        with patch.object(mc.Calibration,'load',side_effect=mc.CalibrationError('bad file')):
            with self.assertRaisesRegex(mc.CalibrationError,'bad file'):
                gui.load_startup_calibration(True,True)
            self.assertEqual(gui.load_startup_calibration(False),(None,False))

    def test_screenshot_target_from_upright_converges_and_has_checked_path(self):
        cal,_=gui.load_startup_calibration(True)
        m=ik.model(gui.GEOMETRY,cal);start=ik.q_to_servo(cal,np.deg2rad([90,90,0,0]),cal.claw_open)
        target=np.array([100.,100.,50.])
        q,r=ik.inverse_kinematics(target,m,ik.servo_to_q(cal,start))
        self.assertTrue(r['success'],r['status']);self.assertLess(r['error']*1000,.1)
        poses,miss=ik.plan_motion(m,cal,start,target,r)
        self.assertGreater(len(poses),0);self.assertLess(miss,gui.MAX_MISS_MM)
        previous=start
        for pose in poses:
            ik.validate_servo_leg(m,cal,previous,pose)
            previous=pose

    def test_reproduces_screenshot_failure_without_altering_legacy_mapping(self):
        cal=screenshot_calibration();m=ik.model(gui.GEOMETRY,cal)
        _,r=ik.inverse_kinematics([100,100,50],m,np.deg2rad([90,90,0,0]))
        self.assertFalse(r['success']);self.assertEqual(r['status'],'stalled')
        self.assertAlmostEqual(r['error']*1000,93.230517,places=4)
        # This is a data-reference issue; do not silently rewrite legacy files.
        np.testing.assert_allclose(np.rad2deg(m.limits[2:]),[[0,180],[0,180]])

    def test_synthetic_reload_cannot_silently_switch_to_saved_calibration(self):
        app=gui.ArmApp.__new__(gui.ArmApp);app.synthetic=True;messages=[];app.log=messages.append
        with patch.object(mc.Calibration,'load') as load:
            app.reload()
        load.assert_not_called();self.assertIn('--use-calibration',messages[0])


if __name__=='__main__':unittest.main()
