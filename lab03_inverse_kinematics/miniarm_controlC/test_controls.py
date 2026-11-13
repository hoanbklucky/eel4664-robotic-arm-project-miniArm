"""Mode presentation, hardware initialization gates and acknowledged STOP."""
import unittest
import numpy as np
import miniarm_controlC as gui
from test_auto_simulation import app_without_window


class Widget:
    def __init__(self,value=''):
        self.options={'state':'normal'};self.value=str(value)
    def config(self,**options):self.options.update(options)
    def set(self,value):self.value=str(value)
    def get(self):return self.value


def controls_app(simulate=True):
    app=app_without_window(simulate)
    app.mode_buttons={mode:Widget() for mode in gui.MODES}
    for name in ('init_button','manual_button','calculate_button','move_button','save_button','reload_button','stop_button'):
        setattr(app,name,Widget())
    app.grip_buttons=[Widget(),Widget()];app.sliders={i:Widget(90) for i in range(5)}
    app.entries=[Widget(v) for v in (100,100,50)];app.mode_status=Widget()
    return app


class ControlTests(unittest.TestCase):
    def test_hardware_motion_controls_locked_until_init_ack(self):
        app=controls_app(False);app.poll()
        for w in [*app.mode_buttons.values(),*app.sliders.values(),*app.entries,
                  app.manual_button,app.calculate_button,app.move_button,app.save_button,*app.grip_buttons]:
            self.assertEqual(w.options['state'],'disabled')
        self.assertEqual(app.init_button.options['state'],'normal')
        self.assertEqual(app.stop_button.options['state'],'normal')
        self.assertEqual(app.reload_button.options['state'],'normal')
        app.set_mode('I');app.link.send.assert_not_called()
        app.initialize();self.assertFalse(app.initialized)
        self.assertEqual(app.mode_buttons['A'].options['state'],'disabled')
        app.inbox.put('MODE A');app.inbox.put('OK');app.poll()
        self.assertTrue(app.initialized)
        self.assertEqual(app.manual_button.options['state'],'normal')
        self.assertEqual(app.mode_buttons['A'].options['relief'],'sunken')

    def test_active_mode_highlight_and_mode_specific_buttons(self):
        app=controls_app();app.poll()
        self.assertEqual(app.mode,'I');self.assertIn('IK CONTROL',app.mode_status.get())
        self.assertEqual(app.mode_buttons['I'].options['bg'],gui.MODES['I'][1])
        self.assertEqual(app.calculate_button.options['state'],'normal')
        self.assertEqual(app.manual_button.options['state'],'disabled')
        app.set_mode('A');app.poll()
        self.assertEqual(app.mode_buttons['A'].options['relief'],'sunken')
        self.assertEqual(app.mode_buttons['I'].options['relief'],'raised')
        self.assertEqual(app.manual_button.options['state'],'normal')
        self.assertEqual(app.calculate_button.options['state'],'disabled')

    def test_stop_cancels_queue_freezes_emulator_and_requires_new_preview(self):
        app=controls_app();app.poll();app.calculate();self.assertIsNotNone(app.preview)
        app.move();self.assertTrue(app.busy)
        app.link.last_step=0;app.poll()  # execute part of the command path
        app.stop();frozen=list(app.link.cur)
        self.assertIsNone(app.preview);self.assertEqual(app.plan,[]);self.assertIsNone(app.pending)
        self.assertTrue(app.stop_pending)
        self.assertEqual(app.calculate_button.options['state'],'disabled')
        self.assertIn('awaiting',app.mode_status.get())
        app.poll();self.assertFalse(app.stop_pending);self.assertTrue(app.stopped)
        self.assertTrue(app.link.on)  # STOP is hold, not power removal
        for _ in range(5):app.link.last_step=0;app.poll()
        self.assertEqual(app.link.cur,frozen)
        self.assertIn('STOPPED',app.mode_status.get())
        self.assertEqual(app.move_button.options['state'],'disabled')

    def test_knob_stop_exits_knob_mode_and_suppresses_automatic_retake(self):
        app=controls_app();app.poll();app.set_mode('K');app.poll()
        app.stop();app.poll()
        self.assertEqual(app.mode,'A');self.assertEqual(app.link.mode,'A')
        app.knob_baseline=[0]*5
        app.inbox.put('K 1023 1023 1023 1023 1023');app.poll()
        self.assertEqual(app.mode,'A');self.assertTrue(app.stopped)
        app.set_mode('K');app.poll();self.assertFalse(app.stopped)
        self.assertEqual(app.mode,'K')

    def test_knob_takeover_switches_after_stop_confirmation(self):
        app=controls_app();app.poll();app.knob_baseline=[0]*5
        app.inbox.put('K 1023 1023 1023 1023 1023');app.poll()
        self.assertEqual(app.mode,'K');self.assertEqual(app.link.mode,'K')
        self.assertFalse(app.stop_pending);self.assertFalse(app.knob_takeover_pending)

    def test_reset_relocks_hardware_and_stop_waits_for_position_report(self):
        app=controls_app(False);app.poll();app.initialize();app.inbox.put('OK');app.poll()
        app.stop();app.inbox.put('STOPPED');app.poll()
        self.assertTrue(app.stop_pending)
        app.inbox.put('P 40 90 90 90 90');app.poll();self.assertFalse(app.stop_pending)
        app.inbox.put('READY');app.poll();self.assertFalse(app.initialized)
        self.assertEqual(app.manual_button.options['state'],'disabled')


if __name__=='__main__':unittest.main()
