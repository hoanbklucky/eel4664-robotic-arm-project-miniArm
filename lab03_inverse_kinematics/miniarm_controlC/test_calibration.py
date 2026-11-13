"""Direct DH calibration and backward compatibility; no physical commands."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
import miniarm_calibration as mc
import miniarm_calibrate as form
import miniarm_ik as ik


def legacy_data():
    return dict(joints={name:dict(points=[[0,0],[45,40],[90,90],[135,140],[180,180]],min=10,max=170)
                       for name in mc.JOINTS},
                claw=dict(open=40,closed=90,min=40,max=90))


class Entry:
    def __init__(self,text=''):self.value=str(text)
    def get(self):return self.value
    def delete(self,*args):self.value=''
    def insert(self,index,text):self.value=str(text)


class Canvas:
    def __init__(self):self.items=[]
    def delete(self,*args):self.items.clear()
    def create_rectangle(self,*c,**o):self.items.append(('rectangle',c,o))
    def create_line(self,*c,**o):self.items.append(('line',c,o))
    def create_text(self,*c,**o):self.items.append(('text',c,o))
    def create_oval(self,*c,**o):self.items.append(('oval',c,o))


def form_without_window():
    app=form.CalibrateApp.__new__(form.CalibrateApp)
    app.entries={name:[(Entry(),Entry()) for _ in range(form.ROWS)] for name in mc.JOINTS}
    app.ranges={name:(Entry(),Entry()) for name in mc.JOINTS}
    app.claw={name:Entry() for name in ('open','closed','min','max')}
    app.plots={name:Canvas() for name in mc.JOINTS}
    app.say=lambda *args,**kwargs:None
    return app


class CalibrationTests(unittest.TestCase):
    def test_legacy_conversion_preserves_every_raw_angle_and_roundtrip(self):
        data=legacy_data();before=copy.deepcopy(data);old=mc.Calibration(data)
        converted=old.as_joint_data();new=mc.Calibration(converted)
        self.assertEqual(data,before);self.assertEqual(converted['angle_convention'],'joint')
        for name in mc.JOINTS:
            for raw in range(181):
                q=old.servo_to_q(name,raw)
                self.assertAlmostEqual(q,new.servo_to_q(name,raw),places=11)
                self.assertEqual(old.q_to_servo(name,q),new.q_to_servo(name,q))
        self.assertEqual(old.all_limits(),new.all_limits())
        self.assertEqual(old.middle_pose(),new.middle_pose())

    def test_direct_wrist_example_and_negative_angle(self):
        data=mc.Calibration(legacy_data()).as_joint_data()
        data['joints']['wrist']=dict(points=[[60,30],[80,15],[100,0],[120,-15],[140,-30]],min=60,max=140)
        cal=mc.Calibration(data)
        self.assertEqual(cal.q_to_servo('wrist',-22.5),130)
        self.assertEqual(cal.q_to_servo('wrist',0),100)
        self.assertEqual(cal.servo_to_q('wrist',130),-22.5)
        self.assertFalse(mc.check(data)[0])

    def test_old_and_new_files_load_with_identical_model_and_startup(self):
        old=mc.Calibration(legacy_data());new=mc.Calibration(old.as_joint_data())
        with tempfile.TemporaryDirectory() as temp:
            for name,cal in [('old',old),('new',new)]:
                path=Path(temp)/(name+'.json');mc.Calibration.save(cal.data,path)
                loaded=mc.Calibration.load(path)
                self.assertEqual(loaded.angle_convention,cal.angle_convention)
                np.testing.assert_allclose(ik.servo_to_q(loaded,[40,150,150,120,90]),
                                           ik.servo_to_q(cal,[40,150,150,120,90]))
        geometry=dict(H=120.,L1=56.5,L2=61.5,L3=85.)
        a,b=ik.model(geometry,old),ik.model(geometry,new)
        np.testing.assert_allclose(a.limits,b.limits)
        np.testing.assert_allclose(a.q0,b.q0)
        self.assertEqual(ik.q_to_servo(old,a.q0,40),ik.q_to_servo(new,b.q0,40))

    def test_form_writes_joint_metadata_and_retains_signed_degrees(self):
        cal=mc.Calibration(legacy_data());app=form_without_window()
        app.fill_form(cal.as_joint_data());data,errors=app.read_form()
        self.assertFalse(errors)
        saved=form.CalibrateApp._as_ints(data)
        self.assertEqual(saved['angle_convention'],'joint')
        self.assertLess(saved['joints']['forearm']['points'][-1][1],0)
        a,b=mc.Calibration(saved),cal
        for name in mc.JOINTS:
            for raw in (10,50,90,130,170):
                self.assertAlmostEqual(a.servo_to_q(name,raw),b.servo_to_q(name,raw))

    def test_form_graph_labels_negative_q_and_rejects_nonfinite(self):
        app=form_without_window();app.fill_form(mc.Calibration(legacy_data()).as_joint_data())
        app.draw('wrist')
        labels=[o['text'] for k,c,o in app.plots['wrist'].items if k=='text']
        self.assertIn('-90',labels);self.assertIn('Raw servo command',labels)
        app.entries['wrist'][0][1].value='nan'
        _,errors=app.read_form();self.assertTrue(errors)
        # Drawing a partly entered/invalid value does not create NaN coordinates.
        app.draw('wrist')
        for _,coords,_ in app.plots['wrist'].items:self.assertTrue(np.isfinite(coords).all())

    def test_unknown_convention_and_bad_numbers_are_rejected(self):
        data=legacy_data();data['angle_convention']='unknown'
        self.assertTrue(mc.check(data)[0])
        with self.assertRaises(mc.CalibrationError):mc.Calibration(data)
        for bad in (float('nan'),float('inf')):
            data=legacy_data();data['joints']['arm']['points'][0][1]=bad
            self.assertTrue(mc.check(data)[0])
            data=legacy_data();data['claw']['open']=bad
            self.assertTrue(mc.check(data)[0])


if __name__=='__main__':unittest.main()
