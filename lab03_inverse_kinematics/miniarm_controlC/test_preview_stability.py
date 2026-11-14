"""A displayed calculation must outlive invalidated executable motion."""
import unittest
import numpy as np
import miniarm_controlC as gui
import miniarm_ik as ik
from test_controls import controls_app


def coordinates(app,tag):
    return [tuple(c) for kind,c,options in app.canvas.items if options.get('tags')==tag]


class PreviewStabilityTests(unittest.TestCase):
    def app(self):
        app=controls_app();app.poll();app.calculate()
        self.assertIsNotNone(app.preview,app.log.call_args_list)
        return app

    def test_input_edit_invalidates_move_but_preserves_drawing_and_curve(self):
        app=self.app();plot=app._preview_plot
        purple=coordinates(app,'preview_link');target=coordinates(app,'target_marker')
        errors=plot[2]['errors'].copy()
        app.entries[0].value='-40';app.invalidate()
        self.assertIsNone(app.preview);self.assertEqual(app.move_button.options['state'],'disabled')
        np.testing.assert_array_equal(app._preview_plot[0],plot[0])
        np.testing.assert_array_equal(app._preview_plot[2]['errors'],errors)
        self.assertEqual(coordinates(app,'preview_link'),purple)
        self.assertEqual(coordinates(app,'target_marker'),target)
        app.calculate()
        self.assertIsNotNone(app.preview)
        self.assertNotEqual(coordinates(app,'target_marker'),target)

    def test_live_servo_reports_do_not_change_purple_pose_target_or_scale(self):
        app=self.app()
        purple=coordinates(app,'preview_link');target=coordinates(app,'target_marker')
        axes={tag:coordinates(app,tag) for tag in ('world_axis_x','world_axis_y','world_axis_z')}
        app.positions=ik.q_to_servo(app.cal,app.m.q0,app.positions[0])
        app.update_position()
        self.assertIsNone(app.preview)  # start changed, so old move is forbidden
        self.assertEqual(coordinates(app,'preview_link'),purple)
        self.assertEqual(coordinates(app,'target_marker'),target)
        for tag,value in axes.items():self.assertEqual(coordinates(app,tag),value)

    def test_move_updates_gray_estimate_without_erasing_calculation(self):
        app=self.app();plot=app._preview_plot
        purple=coordinates(app,'preview_link');target=coordinates(app,'target_marker')
        app.move();self.assertTrue(app.busy);self.assertIsNotNone(app.link.timed)
        app.link.timed[2]-=app.link.timed[3]/2;app.poll()
        self.assertIsNone(app.preview)
        np.testing.assert_array_equal(app._preview_plot[0],plot[0])
        self.assertEqual(coordinates(app,'preview_link'),purple)
        self.assertEqual(coordinates(app,'target_marker'),target)
        self.assertGreater(len(coordinates(app,'current_link')),0)

    def test_stop_keeps_display_but_reset_explicitly_clears_it(self):
        app=self.app();app.stop();app.poll()
        self.assertIsNotNone(app._preview_plot)
        self.assertGreater(len(coordinates(app,'preview_link')),0)
        app.inbox.put('READY');app.poll()
        self.assertIsNone(app._preview_plot)
        self.assertEqual(coordinates(app,'target_marker'),[])

    def test_workspace_bounds_do_not_depend_on_current_configuration(self):
        app=self.app();bounds=gui.preview_workspace_points(app.m)
        before=bounds.copy()
        app.positions=ik.q_to_servo(app.cal,np.deg2rad([120,50,-70,-40]),40)
        np.testing.assert_array_equal(gui.preview_workspace_points(app.m),before)
        for q in (app.m.q0,np.deg2rad([90,90,0,0]),np.deg2rad([45,0,0,0])):
            points=ik.fk(app.m,q)[1]*1000
            self.assertTrue(np.all(points>=bounds.min(axis=0)-1e-8))
            self.assertTrue(np.all(points<=bounds.max(axis=0)+1e-8))


if __name__=='__main__':unittest.main()
