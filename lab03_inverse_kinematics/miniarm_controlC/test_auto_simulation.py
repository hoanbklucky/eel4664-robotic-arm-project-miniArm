"""Only the emulator initializes automatically; physical hardware stays gated."""
import queue
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import miniarm_controlC as gui
from test_preview import recorded_app


def app_without_window(simulate=True):
    app=recorded_app();app.inbox=queue.Queue()
    app.link=gui.SimulatedArm(app.inbox) if simulate else SimpleNamespace(simulated=False,tick=Mock(),send=Mock())
    if not simulate:app.inbox.put('READY')
    app.root=SimpleNamespace(after=Mock())
    app.ready=False;app.initialized=False;app.busy=False;app.pending=None;app.plan=[];app.active=None
    app.preview=None;app.knobs=None;app.knob_baseline=None;app.mode='A';app.started=0
    app.sliders={i:Mock() for i in range(5)};app.readout=Mock();app.move_button=Mock();app.log=Mock()
    app.commanded=[90]*5
    app.entries=[SimpleNamespace(get=lambda value=str(x):value) for x in [100,100,50]]
    return app


class AutoSimulationTests(unittest.TestCase):
    def test_ready_allows_calculate_without_upright_or_mode_click(self):
        app=app_without_window();app.poll()
        self.assertTrue(app.initialized);self.assertTrue(app.link.on)
        self.assertFalse(app.busy);self.assertEqual(app.mode,'I')
        before=list(app.positions);app.calculate()
        self.assertIsNotNone(app.preview,app.log.call_args_list)
        self.assertEqual(app.positions,before)  # Calculate is still preview only
        self.assertFalse(app.busy)
        app.move_button.config.assert_called_with(state='normal')

    def test_hardware_ready_does_not_initialize_or_send_anything(self):
        app=app_without_window(False);app.poll()
        self.assertTrue(app.ready);self.assertFalse(app.initialized)
        self.assertEqual(app.mode,'A');app.link.send.assert_not_called()
        app.calculate();self.assertIsNone(app.preview)
        self.assertIn('Initialize',app.log.call_args.args[0])

    def test_emulator_reset_restores_direct_preview(self):
        app=app_without_window();app.poll();app.calculate();self.assertIsNotNone(app.preview)
        app.inbox.put('READY');app.poll()
        self.assertTrue(app.initialized);self.assertEqual(app.mode,'I')
        self.assertIsNone(app.preview);self.assertFalse(app.busy)


if __name__=='__main__':unittest.main()
