"""Predictable upright action, visible calculation feedback, calibration edits."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch
import miniarm_controlC as gui
import miniarm_ik as ik
import miniarm_calibrate as form
from test_controls import controls_app,Widget
from test_calibration import form_without_window


def control():
    app=controls_app();app.action_hint=Widget();app.result_status=Widget();app.result_label=Widget()
    app.poll();return app


class InteractionTests(unittest.TestCase):
    def test_upright_preserves_ik_and_claw_and_finishes_ready_for_calculate(self):
        app=control();app.positions=ik.q_to_servo(app.cal,app.m.q0,75)
        app.link.cur=list(app.positions);app.link.tgt=list(app.positions)
        app.initialize();self.assertEqual(app.mode,'I');self.assertTrue(app.busy)
        for _ in range(1000):
            if app.link.timed is not None:app.link.timed[2]-=.05
            app.link.last_step=0;app.poll()
            if not app.busy:break
        self.assertFalse(app.busy);self.assertEqual(app.mode,'I');self.assertEqual(app.link.mode,'I')
        self.assertEqual(app.positions[0],75)
        self.assertEqual(app.calculate_button.options['state'],'normal')
        self.assertEqual(app.mode_buttons['I'].options['relief'],'sunken')
        app.calculate();self.assertIsNotNone(app.preview)

    def test_upright_preserves_app_but_does_not_silently_leave_knob_mode(self):
        app=control();app.set_mode('A');app.poll();app.initialize();app.link.timed[2]-=20;app.link.last_step=0;app.poll()
        self.assertEqual(app.mode,'A');self.assertEqual(app.link.mode,'A')
        app.set_mode('K');app.poll();self.assertEqual(app.mode,'K');before=list(app.link.cur)
        self.assertEqual(app.init_button.options['state'],'disabled')
        app.initialize();self.assertEqual(app.mode,'K');self.assertFalse(app.busy)
        self.assertEqual(app.link.cur,before);self.assertIn('Choose APP or IK',app.feedback_message)
        self.assertIn('Choose APP or IK',app.action_hint.get())

    def test_success_failure_and_invalid_field_are_visible(self):
        app=control();app.calculate()
        self.assertEqual(app.feedback_kind,'success');self.assertIn('Preview ready',app.result_status.get())
        self.assertIn('Move Arm',app.action_hint.get())
        app.entries[0].value='not a number';app.calculate()
        self.assertEqual(app.feedback_kind,'error');self.assertIn('X must be',app.result_status.get())
        self.assertIsNone(app.preview);self.assertEqual(app.move_button.options['state'],'disabled')
        app.entries[0].value='1000';app.entries[1].value='1000';app.entries[2].value='1000'
        app.calculate();self.assertIn('No move prepared',app.result_status.get())
        self.assertIsNone(app.preview)

    def test_calibration_tracks_unsaved_edits_and_save_metadata(self):
        app=form_without_window();app.fill_form(ik.demo_calibration().data)
        app.edited('forearm');self.assertTrue(app.dirty);self.assertIn('Unsaved',app.summary_text)
        with tempfile.TemporaryDirectory() as temp:
            path=Path(temp)/'calibration.json'
            with patch.object(form,'CALIBRATION_FILE',str(path)):
                self.assertTrue(app.save())
            data=json.loads(path.read_text());self.assertEqual(data['angle_convention'],'joint')
            self.assertFalse(app.dirty);self.assertIn('Saved',app.summary_text)
        app.edited()
        with patch.object(form.mc.Calibration,'save',side_effect=OSError('disk full')):
            self.assertFalse(app.save())
        self.assertTrue(app.dirty);self.assertIn('Save failed',app.summary_text)

    def test_calibration_close_cancel_and_failed_save_keep_measurements(self):
        app=form_without_window();app.root=Mock();app.dirty=True
        with patch.object(form.messagebox,'askyesnocancel',return_value=None):app.close()
        app.root.destroy.assert_not_called()
        with patch.object(form.messagebox,'askyesnocancel',return_value=True),patch.object(app,'save',return_value=False):
            app.close()
        app.root.destroy.assert_not_called()
        with patch.object(form.messagebox,'askyesnocancel',return_value=False):app.close()
        app.root.destroy.assert_called_once()

    def test_calibration_reload_cancel_preserves_entered_rows(self):
        app=form_without_window();app.root=Mock();app.fill_form(ik.demo_calibration().data)
        app.entries['forearm'][0][1].value='88';app.edited()
        with patch.object(form.os.path,'exists',return_value=True),\
             patch.object(form.mc.Calibration,'load',return_value=ik.demo_calibration()),\
             patch.object(form.messagebox,'askyesno',return_value=False):
            app.load()
        self.assertEqual(app.entries['forearm'][0][1].get(),'88');self.assertTrue(app.dirty)
        app.loading=True;app.dirty=False;app.edited();self.assertFalse(app.dirty)


if __name__=='__main__':unittest.main()
