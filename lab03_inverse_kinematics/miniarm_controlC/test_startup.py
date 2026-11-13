"""Startup posture and motion guards without a Tk window or hardware."""
import unittest
from unittest.mock import Mock, patch
import numpy as np
import miniarm_controlC as gui
import miniarm_ik as ik


class StartupTests(unittest.TestCase):
    def app(self):
        app=gui.ArmApp.__new__(gui.ArmApp)
        app.cal=ik.demo_calibration();app.m=ik.model(gui.GEOMETRY,app.cal)
        app.ready=True;app.busy=False;app.initialized=False
        app.positions=ik.q_to_servo(app.cal,app.m.q0,75)
        app.send=Mock();app.launch=Mock();app.invalidate=Mock();app.log=Mock()
        app.sliders={i:Mock() for i in range(5)}
        return app

    def test_fixed_upright_not_sliders_or_ik_seed(self):
        app=self.app();app.m.q0=np.deg2rad([110,70,-80,-40])
        np.testing.assert_allclose(np.rad2deg(ik.servo_to_q(app.cal,app.start_pose())),[90,90,0,0])
        app.initialize()
        app.launch.assert_called_once_with([app.start_pose()],command='INIT')
        for slider in app.sliders.values():slider.get.assert_not_called()

    def test_unavailable_upright_sends_nothing_and_does_not_clip(self):
        app=self.app();app.m.limits[1,1]=np.deg2rad(80)
        app.initialize();app.send.assert_not_called();app.launch.assert_not_called()
        self.assertIn('outside calibrated limits',app.log.call_args.args[0])

    def test_already_initialized_checks_path_and_preserves_claw(self):
        app=self.app();app.initialized=True
        with patch.object(ik,'validate_servo_leg') as check:
            app.initialize()
            target=app.start_pose();target[0]=75
            check.assert_called_once_with(app.m,app.cal,app.positions,target)
            app.launch.assert_called_once_with([target])
        app=self.app();app.initialized=True
        with patch.object(ik,'validate_servo_leg',side_effect=ValueError('blocked path')):
            app.initialize()
        app.send.assert_not_called();app.launch.assert_not_called()

    def test_missing_calibration_is_explicit_raw_neutral(self):
        app=self.app();app.cal=None
        self.assertEqual(app.start_pose(),[90]*5)
        app.init_button=Mock();app.init_text=Mock();app.update_init_text()
        self.assertIn('uncalibrated',app.init_button.config.call_args.kwargs['text'])
        self.assertIn('angles unknown',app.init_text.set.call_args.args[0])


if __name__=='__main__':unittest.main()
