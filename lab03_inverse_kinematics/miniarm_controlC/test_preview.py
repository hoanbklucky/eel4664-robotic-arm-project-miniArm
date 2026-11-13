"""World axes/camera regression tests, including the real Tk preview."""
import unittest
import os
from types import SimpleNamespace
import numpy as np
import miniarm_controlC as gui
import miniarm_ik as ik


class ProjectionTests(unittest.TestCase):
    def test_basis_is_uniform_and_right_handed_from_above(self):
        for view in gui.PREVIEW_CAMERAS:
            right,up,camera=gui.preview_camera_basis(view)
            basis=np.stack([right,up,camera])
            np.testing.assert_allclose(basis@basis.T,np.eye(3),atol=1e-12)
            np.testing.assert_allclose(np.cross(right,up),camera,atol=1e-12)
            self.assertGreaterEqual(camera[2],0)
            project=gui.fit_preview_projection(np.eye(3)*100,view,(20,60,400,280))
            origin=np.array(project(np.zeros(3)))
            a=np.linalg.norm(np.array(project(right*100))-origin)
            b=np.linalg.norm(np.array(project(up*100))-origin)
            self.assertAlmostEqual(a,b)

    def test_default_rear_right_matches_frame_illustration(self):
        view='Rear right (Arduino side)'
        right,up,camera=gui.preview_camera_basis(view)
        expected=np.array([1.,-1.,.8]);expected/=np.linalg.norm(expected)
        np.testing.assert_allclose(camera,expected,atol=1e-12)
        p=gui.fit_preview_projection(np.eye(3)*100,view,(20,60,400,280))
        o=np.array(p([0,0,0]));x=np.array(p([100,0,0]));y=np.array(p([0,100,0]));z=np.array(p([0,0,100]))
        self.assertGreater(x[0],o[0]);self.assertGreater(y[0],o[0])
        self.assertLess(y[1],o[1]);self.assertLess(z[1],o[1])

    def test_rear_and_top_views_have_expected_axes(self):
        for view in ('Rear (Arduino side)','Top'):
            p=gui.fit_preview_projection(np.eye(3)*100,view,(20,60,400,280))
            o=np.array(p([0,0,0]));x=np.array(p([100,0,0]));y=np.array(p([0,100,0]))
            self.assertGreater(x[0],o[0]);self.assertLess(y[1],o[1])
            self.assertAlmostEqual(x[1],o[1]);self.assertAlmostEqual(y[0],o[0])


class RecordingCanvas:
    """Record the actual draw() calls without requiring a window server."""
    def __init__(self):self.items=[]
    def delete(self,*args):self.items.clear()
    def winfo_width(self):return 840
    def winfo_height(self):return 340
    def record(self,kind,coords,options):
        self.items.append((kind,coords,options));return len(self.items)
    def create_line(self,*coords,**options):return self.record('line',coords,options)
    def create_text(self,*coords,**options):return self.record('text',coords,options)
    def create_oval(self,*coords,**options):return self.record('oval',coords,options)
    def create_polygon(self,*coords,**options):return self.record('polygon',coords,options)


def recorded_app(view='Rear right (Arduino side)'):
    app=gui.ArmApp.__new__(gui.ArmApp)
    app.cal=ik.demo_calibration();app.m=ik.model(dict(H=120.,L1=56.5,L2=61.5,L3=85.),app.cal)
    app.initialized=True;app.positions=ik.q_to_servo(app.cal,app.m.q0,40)
    app.canvas=RecordingCanvas();app.preview_view=SimpleNamespace(get=lambda:view)
    app._preview_plot=None
    return app


class CanvasTests(unittest.TestCase):
    def test_small_window_does_not_invert_projection(self):
        app=recorded_app();app.canvas.winfo_width=lambda:300;app.canvas.winfo_height=lambda:120
        app.draw()
        texts=[o['text'] for k,c,o in app.canvas.items if k=='text']
        self.assertEqual(texts,['Enlarge the window to show the 3D preview.'])

    def test_actual_renderer_uses_same_projection_for_arm_and_target(self):
        app=recorded_app();target=np.array([40.,100.,150.])
        q,r=ik.inverse_kinematics(target,app.m,app.m.q0)
        self.assertTrue(r['success'])
        app.draw(q,target,r)
        items=app.canvas.items
        current=[c for k,c,o in items if o.get('tags')=='current_link']
        preview=[c for k,c,o in items if o.get('tags')=='preview_link']
        marker=[c for k,c,o in items if o.get('tags')=='target_marker']
        self.assertTrue(current);self.assertTrue(preview);self.assertEqual(len(marker),2)
        target_pixel=(np.array(marker[0][:2])+marker[0][2:])/2
        self.assertLess(np.linalg.norm(target_pixel-np.array(preview[-1][2:])),.1)
        self.assertGreater(np.linalg.norm(np.array(current[-1][2:])-preview[-1][2:]),1.)
        for coords in current+preview+marker:
            self.assertTrue(np.isfinite(coords).all())
            self.assertTrue(all(0<=x<=420 for x in coords[::2]))
            self.assertTrue(all(50<=y<=300 for y in coords[1::2]))
        texts=[o['text'] for k,c,o in items if k=='text']
        self.assertTrue(any('Gray: current estimate' in t for t in texts))
        self.assertIn('World frame {0}: X right, Y forward, Z up',texts)

    def test_all_camera_views_redraw_without_changing_model_or_commands(self):
        for view in gui.PREVIEW_CAMERAS:
            app=recorded_app(view);before=list(app.positions)
            q0=app.m.q0.copy();app.draw()
            self.assertEqual(app.positions,before)
            np.testing.assert_array_equal(app.m.q0,q0)
            self.assertTrue(app.canvas.items)
            if view=='Top':
                texts=[o['text'] for k,c,o in app.canvas.items if k=='text']
                self.assertIn('Z up (toward viewer)',texts)


@unittest.skipUnless(os.environ.get('DISPLAY') or os.name=='nt','Interactive Tk test requires a display')
class PreviewGuiTests(unittest.TestCase):
    def test_camera_change_keeps_preview_and_does_not_move(self):
        import tkinter as tk
        root=tk.Tk();root.withdraw();app=gui.ArmApp(root,simulate=True)
        try:
            app.poll();app.initialize();app.poll();app.set_mode('I');app.poll()
            target=np.array([40.,100.,150.])
            for e,v in zip(app.entries,target):e.delete(0,'end');e.insert(0,str(v))
            before=list(app.positions);app.calculate()
            self.assertIsNotNone(app.preview,app.terminal.get('1.0','end'))
            q=app.preview['q'].copy()
            self.assertGreater(len(app.canvas.find_withtag('current_link')),0)
            self.assertGreater(len(app.canvas.find_withtag('preview_link')),0)
            for view in gui.PREVIEW_CAMERAS:
                app.preview_view.set(view);app.redraw_preview()
                np.testing.assert_array_equal(app.preview['q'],q)
                self.assertEqual(app.positions,before);self.assertFalse(app.busy)
                self.assertEqual(len(app.canvas.find_withtag('target_marker')),2)
            app.invalidate()
            self.assertIsNone(app._preview_plot)
            self.assertEqual(len(app.canvas.find_withtag('target_marker')),0)
        finally:app.close()


if __name__=='__main__':unittest.main()
