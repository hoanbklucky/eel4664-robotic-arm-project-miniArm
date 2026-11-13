"""Offline numerical, motion-protocol and GUI regression tests. No serial port."""
import unittest
import os
import numpy as np
import miniarm_ik as ik
import miniarm_calibration as mc
import miniarm_controlC as gui

G=dict(H=120.,L1=56.5,L2=61.5,L3=85.)

class IntegrationTests(unittest.TestCase):
    def setUp(self):
        self.cal=ik.demo_calibration(); self.m=ik.model(G,self.cal)
        self.start=ik.q_to_servo(self.cal,self.m.q0,40)

    def test_reference_axes_and_jacobian(self):
        for base,expected in [(0,[203,0,120]),(90,[0,203,120]),(180,[-203,0,120])]:
            np.testing.assert_allclose(ik.forward_kinematics([base,0,0,0],G)['tool'],expected,atol=1e-10)
        q=[110,70,-80,-40]
        np.testing.assert_allclose(ik.jacobian(q,G),ik.jacobian(q,G,'finite'),atol=1e-6)
        # Independent explicit FK, not the same transformation code.
        b,a,e,w=np.deg2rad(q)
        r=56.5*np.cos(a)+61.5*np.cos(a+e)+85*np.cos(a+e+w)
        z=120+56.5*np.sin(a)+61.5*np.sin(a+e)+85*np.sin(a+e+w)
        np.testing.assert_allclose(ik.forward_kinematics(q,G)['tool'],[r*np.cos(b),r*np.sin(b),z],atol=1e-10)

    def test_solver_and_quantized_execution(self):
        target=ik.position(self.m,np.deg2rad([110,70,-80,-40]))*1000
        q,r=ik.inverse_kinematics(target,self.m,ik.servo_to_q(self.cal,self.start))
        self.assertTrue(r['success'],r['status'])
        poses,miss=ik.plan_motion(self.m,self.cal,self.start,target,r)
        self.assertGreater(len(poses),0); self.assertLess(miss,10)
        self.assertTrue(all(p[0]==40 for p in poses))
        prev=self.start
        for pose in poses:
            for state in ik.sequential_states(prev,pose):
                self.assertTrue(ik.clearance(self.m,ik.servo_to_q(self.cal,state))[0]>0)
            prev=pose

    def test_sequential_order_matches_firmware(self):
        start=[40,90,90,90,90]; end=[41,91,89,92,88]
        states=list(ik.sequential_states(start,end))
        self.assertEqual(states[0],[40,90,90,90,89])
        self.assertEqual(states[-1],end)
        self.assertEqual(len(states),7)

    def test_invalid_targets_and_stale_start(self):
        with self.assertRaises(ValueError): ik.inverse_kinematics([np.nan,0,0],self.m,self.m.q0)
        q,r=ik.inverse_kinematics([1000,1000,1000],self.m,self.m.q0)
        self.assertFalse(r['success'])
        _,r=ik.inverse_kinematics(ik.position(self.m,self.m.q0)*1000,self.m,self.m.q0)
        changed=list(self.start); changed[4]+=1
        with self.assertRaises(ValueError): ik.plan_motion(self.m,self.cal,changed,ik.position(self.m,self.m.q0)*1000,r)

    def test_calibration_roundtrip_and_limits(self):
        q=ik.servo_to_q(self.cal,self.start)
        np.testing.assert_allclose(q,self.m.q0,atol=1e-12)
        self.assertEqual(ik.q_to_servo(self.cal,q,40),self.start)
        with self.assertRaises(ValueError): ik.q_to_servo(self.cal,np.deg2rad([200,60,-60,-60]),40)
        # Piecewise slope changes do not require linear calibration.
        self.cal.data['joints']['arm']['points']=[[0,0],[80,60],[180,180]]
        cal=mc.Calibration(self.cal.data)
        pose=ik.q_to_servo(cal,self.m.q0,40)
        self.assertLess(abs(ik.servo_to_q(cal,pose)[1]-self.m.q0[1]),.02)

    def test_colliding_execution_rejected(self):
        bad=ik.q_to_servo(self.cal,np.deg2rad([90,0,0,-90]),40)
        with self.assertRaises(ValueError): ik.validate_servo_leg(self.m,self.cal,self.start,bad)

@unittest.skipUnless(os.environ.get('DISPLAY') or os.name=='nt', 'Interactive Tk test requires a display')
class GuiTests(unittest.TestCase):
    def test_preview_does_not_move_then_execution_and_stop(self):
        import tkinter as tk
        root=tk.Tk(); root.withdraw()
        app=gui.ArmApp(root,simulate=True)
        try:
            app.poll(); self.assertTrue(app.initialized)
            self.assertTrue(app.link.on); self.assertEqual(app.mode,'I')
            target=ik.position(app.m,np.deg2rad([110,70,-80,-40]))*1000
            for e,v in zip(app.entries,target): e.delete(0,'end'); e.insert(0,str(v))
            before=list(app.positions); app.calculate()
            self.assertIsNotNone(app.preview,app.terminal.get('1.0','end'))
            self.assertEqual(app.positions,before)
            self.assertFalse(app.busy)
            app.move(); self.assertTrue(app.busy)
            for _ in range(500):
                app.link.last_step=0; app.poll()
                if not app.busy: break
            self.assertFalse(app.busy)
            self.assertLess(np.linalg.norm(ik.position(app.m,ik.servo_to_q(app.cal,app.positions))*1000-target),10)
            app.stop(); self.assertEqual(app.plan,[]); self.assertIsNone(app.preview)
        finally: app.close()

if __name__=='__main__': unittest.main()

