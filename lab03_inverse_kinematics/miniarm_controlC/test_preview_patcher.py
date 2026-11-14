"""Standalone preview patcher preserves custom controls and original bytes."""
import ast
from pathlib import Path
import tempfile
import unittest
from patch_preview_stability import apply,patch_source,MARKER

SOURCE='''from miniarm_version import APP_VERSION, DEVELOPER
import numpy as np
import miniarm_ik as ik
class ArmApp:
    def custom_servo_controls(self):
        return 'keep my buttons and retry solver'
    def invalidate(self):
        self.preview=None
        self._preview_plot=None
        self.draw()
    def draw(self,q=None,target=None,result=None):
        self.draw_error(result)
        current=P=axes=np.zeros((3,3))
        fit_points=np.vstack((current,P,[[0,0,0]],axes))
        if target is not None: fit_points=np.vstack((fit_points,target))
        if unsent is not None: fit_points=np.vstack((fit_points,unsent))
        return fit_points
    def update_position(self):
        self.update_servo_line()
        if self.preview: self.redraw_preview()
        else: self.draw()
    def poll(self):
        self.invalidate()
    def reload(self):
        self.invalidate()
'''

class PreviewPatcherTests(unittest.TestCase):
    def test_custom_layout_controls_and_solver_are_retained(self):
        result=patch_source(SOURCE)
        ast.parse(result)
        self.assertIn("return 'keep my buttons and retry solver'",result)
        self.assertIn('self.draw_error(result)',result)
        self.assertIn('self.update_servo_line()',result)
        self.assertIn('fit_points=preview_workspace_points(self.m,target)',result)
        self.assertIn('self.invalidate(clear_display=True)',result)
        self.assertIn("APP_VERSION = '2.0.2'",result)
        self.assertEqual(patch_source(result),result)

    def test_backup_exact_bytes_and_refuse_unrecognized_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'controller.py'
            raw=SOURCE.replace('\n','\r\n').encode('utf-8')
            path.write_bytes(raw);apply(path)
            self.assertEqual(path.with_name(path.name+'.before_preview_fix').read_bytes(),raw)
            self.assertIn(MARKER,path.read_text())
            old=path.read_bytes();apply(path);self.assertEqual(path.read_bytes(),old)
        with self.assertRaises(ValueError):patch_source('class Other: pass')

    def test_current_repository_controller_also_compiles_after_patch(self):
        source=(Path(__file__).parent/'miniarm_controlC.py').read_text(encoding='utf-8')
        ast.parse(patch_source(source))

if __name__=='__main__':unittest.main()
