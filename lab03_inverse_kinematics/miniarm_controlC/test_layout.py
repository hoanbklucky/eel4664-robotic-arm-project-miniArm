"""Mode-specific controls and protected preview space on actual Tk windows."""
import os
import unittest
from types import SimpleNamespace
import miniarm_controlC as gui


class Panel:
    def __init__(self):self.visible=True
    def grid(self):self.visible=True
    def grid_remove(self):self.visible=False


class ModePanelTests(unittest.TestCase):
    def test_irrelevant_inputs_hidden_and_restored_without_destroying_them(self):
        app=gui.ArmApp.__new__(gui.ArmApp)
        for name in ('manual_panel','ik_panel','timing_panel','ik_actions','claw_actions','result_label'):
            setattr(app,name,Panel())
        app._layout_mode=None;app.feedback_kind='info'
        scrolls=[]
        app.controls_canvas=SimpleNamespace(yview_moveto=scrolls.append)
        app.queue_pane_fit=lambda:None
        manual=app.manual_panel
        for mode,visible in [('I',(False,True,True)),('K',(False,False,False)),('A',(True,False,True))]:
            app.mode=mode;app.update_mode_panels()
            self.assertEqual((app.manual_panel.visible,app.ik_panel.visible,app.timing_panel.visible),visible)
            self.assertEqual(app.ik_actions.visible,mode=='I')
            self.assertEqual(app.claw_actions.visible,mode!='K')
            self.assertIs(app.manual_panel,manual)
        self.assertEqual(scrolls,[0,0,0])
        app.feedback_kind='error';app.update_mode_panels();self.assertTrue(app.result_label.visible)

    def test_preview_space_is_reserved_before_overflowing_controls(self):
        app=gui.ArmApp.__new__(gui.ArmApp)
        calls=[]
        app.preview_tabs=SimpleNamespace(winfo_reqheight=lambda:430)
        app.canvas=SimpleNamespace(winfo_reqheight=lambda:340)
        app.controls_body=SimpleNamespace(winfo_reqheight=lambda:700)
        app.controls_shell=object();app.mode='A';app._pane_fit=None
        app.main_panes=SimpleNamespace(winfo_height=lambda:600,
            paneconfigure=lambda pane,**kwargs:calls.append((pane,kwargs)),
            sash_place=lambda *args:calls.append(('sash',args)))
        app.fit_preview_pane()
        self.assertEqual(calls[0][1]['minsize'],300)
        self.assertEqual(calls[1][1]['height'],292)
        self.assertLess(calls[1][1]['height'],700)  # controls scroll, not the plot
        count=len(calls);app.fit_preview_pane();self.assertEqual(len(calls),count)


@unittest.skipUnless(os.name=='nt' or os.environ.get('DISPLAY'),'Native Tk layout check needs a display')
class NativeLayoutTests(unittest.TestCase):
    def test_preview_at_windows_sizes_scalings_and_all_modes(self):
        import tkinter as tk
        for scaling in (4/3,2.0):
            root=tk.Tk();app=None
            try:
                root.tk.call('tk','scaling',scaling)
                app=gui.ArmApp(root,simulate=True)
                for width,height in ((1100,720),(1366,768),(1900,1040)):
                    root.geometry(f'{width}x{height}')
                    root.update_idletasks();root.update()
                    for mode in ('I','K','A','I'):
                        before=list(app.positions)
                        app.set_mode(mode)
                        root.update_idletasks();root.update()
                        app.fit_preview_pane()
                        root.update_idletasks();root.update()
                        app.redraw_preview()
                        message=f'{width}x{height}, scaling={scaling}, mode={mode}'
                        self.assertGreaterEqual(app.canvas.winfo_height(),180,message)
                        self.assertGreaterEqual(app.canvas.winfo_width(),600,message)
                        self.assertEqual(bool(app.manual_panel.winfo_ismapped()),mode=='A',message)
                        self.assertEqual(bool(app.ik_panel.winfo_ismapped()),mode=='I',message)
                        labels=[app.canvas.itemcget(i,'text') for i in app.canvas.find_all()
                                if app.canvas.type(i)=='text']
                        self.assertFalse(any('Enlarge the window' in text for text in labels),message)
                        self.assertGreater(len(app.canvas.find_withtag('preview_link')),0,message)
                        self.assertEqual(app.positions,before,message)
                        self.assertEqual(app.preview_tabs.index('current'),0,message)
                # The independent log tab cannot consume preview height.
                app.preview_tabs.select(1);root.update()
                self.assertTrue(app.terminal.winfo_ismapped())
                app.preview_tabs.select(0);root.update()
                self.assertGreaterEqual(app.canvas.winfo_height(),180)
            finally:
                if app is not None:app.close()
                else:root.destroy()


if __name__=='__main__':unittest.main()
