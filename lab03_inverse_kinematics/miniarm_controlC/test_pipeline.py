"""Independent frame math, MATLAB configuration, calibration and GUI pipeline."""
import copy
import queue
import re
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import numpy as np
import miniarm_controlC as gui
import miniarm_calibration as mc
import miniarm_ik as ik
from kinematics import mdh
from miniarm_source_model import source_model
from test_preview import recorded_app

ROOT=Path(__file__).resolve().parents[1]


def matrix(text,field):
    match=re.search(r'm\.'+re.escape(field)+r'\s*=\s*(?:deg2rad\()?\[(.*?)\]',text,re.S)
    def value(token):return np.pi/2 if token=='pi/2' else float(token)
    return np.array([[value(v) for v in row.split()] for row in match.group(1).split(';')])


def rotation(axis,t):
    H=np.eye(4);c,s=np.cos(t),np.sin(t)
    H[:3,:3]=[[1,0,0],[0,c,-s],[0,s,c]] if axis=='x' else [[c,-s,0],[s,c,0],[0,0,1]]
    return H


def translation(x=0,y=0,z=0):
    H=np.eye(4);H[:3,3]=[x,y,z];return H


class PipelineTests(unittest.TestCase):
    def test_matlab_gui_model_including_simulation_limits_and_collision(self):
        text=(ROOT/'matlab/miniarm_model.m').read_text();m=ik.model(gui.GEOMETRY,ik.demo_calibration())
        np.testing.assert_allclose(m.mdh,matrix(text,'mdh'),atol=1e-14)
        np.testing.assert_allclose(m.limits,np.deg2rad(matrix(text,'limits')))
        np.testing.assert_allclose(m.q0,np.deg2rad(matrix(text,'q0')).ravel())
        np.testing.assert_allclose(m.base,np.eye(4));self.assertEqual(m.tool[0,3],.085)
        for field in ('edges','pairs'):
            np.testing.assert_array_equal(m.collision[field],matrix(text,'collision.'+field)-1)
        np.testing.assert_allclose(m.collision['radii'],matrix(text,'collision.radii').ravel())
        np.testing.assert_array_equal(m.collision['floor_edges'],matrix(text,'collision.floorEdges').ravel()-1)
        self.assertIn('m.collision.margin = .003;',text);self.assertEqual(m.collision['margin'],.003)
        self.assertIn('m.collision.floorZ = 0;',text);self.assertEqual(m.collision['floor_z'],0.)

    def test_every_frame_tcp_and_independent_xyz_jacobian(self):
        rng=np.random.default_rng(4664);m=source_model();L=np.array([.0565,.0615,.085])
        for _ in range(60):
            q=rng.uniform([0,0,-2.7,-2.7],[np.pi,np.pi,2.7,2.7]);b=q[0];a=np.cumsum(q[1:])
            # Construct each frame from separate elementary transforms.
            T=np.eye(4);frames=[T.copy()]
            for row,t in zip(m.mdh,q):
                length,alpha,d,offset=row
                T=T@rotation('x',alpha)@translation(x=length)@rotation('z',t+offset)@translation(z=d)
                frames.append(T.copy())
            T=T@translation(x=.085);frames.append(T.copy())
            actual,points=ik.fk(m,q)
            np.testing.assert_allclose(actual,T,atol=2e-15)
            np.testing.assert_allclose(points,np.array([f[:3,3] for f in frames]),atol=2e-15)
            r=L@np.cos(a);z=.120+L@np.sin(a)
            np.testing.assert_allclose(actual[:3,3],[r*np.cos(b),r*np.sin(b),z],atol=2e-15)
            dr=np.array([-L[j:]@np.sin(a[j:]) for j in range(3)])
            dz=np.array([L[j:]@np.cos(a[j:]) for j in range(3)])
            expected=np.column_stack(([-r*np.sin(b),r*np.cos(b),0],
                                       np.array([np.cos(b)*dr,np.sin(b)*dr,dz])))
            np.testing.assert_allclose(ik.core_jacobian(m,q),expected,atol=1e-14)
            np.testing.assert_allclose(ik.core_jacobian(m,q,'finite'),expected,atol=1e-9)
            # GUI degree input and mm output must be only unit conversion.
            f=ik.forward_kinematics(np.rad2deg(q),gui.GEOMETRY)
            np.testing.assert_allclose(f['tool'],actual[:3,3]*1000,atol=1e-10)
            np.testing.assert_allclose(ik.jacobian(np.rad2deg(q),gui.GEOMETRY),expected*1000,atol=1e-10)

    def test_general_craig_transform_and_derivative_with_offsets(self):
        row=[.034,-.67,.051,.31];q=.73;h=1e-6
        def independent(t):
            return rotation('x',row[1])@translation(x=row[0])@rotation('z',row[3]+t)@translation(z=row[2])
        A,D=mdh(row,q)
        np.testing.assert_allclose(A,independent(q),atol=1e-15)
        np.testing.assert_allclose(D,(independent(q+h)-independent(q-h))/(2*h),atol=1e-10)

    def test_nonlinear_calibration_equations_and_channel_order(self):
        data=copy.deepcopy(ik.demo_calibration().data)
        data['joints']['base']=dict(points=[[0,180],[90,90],[180,0]],min=0,max=180)
        data['joints']['arm']=dict(points=[[0,0],[60,40],[100,100],[180,180]],min=0,max=180)
        data['joints']['forearm']=dict(points=[[0,160],[60,50],[120,-40],[180,-160]],min=0,max=180)
        data['joints']['wrist']=dict(points=[[0,-160],[70,-20],[120,80],[180,160]],min=0,max=180)
        cal=mc.Calibration(data)
        # Midpoints calculated independently for each segment, not servo=q.
        pose=ik.q_to_servo(cal,np.deg2rad([135,70,5,30]),75)
        self.assertEqual(pose,[75,95,90,80,45])  # S1 claw, S2 wrist, S3 elbow, S4 shoulder, S5 base
        np.testing.assert_allclose(np.rad2deg(ik.servo_to_q(cal,pose)),[135,70,5,30])
        with self.assertRaises(ValueError):ik.q_to_servo(cal,np.deg2rad([181,70,5,30]),75)
        data['joints']['base']['min']=.5
        self.assertTrue(any('integers' in e for e in mc.check(data)[0]))

    def test_gui_calculate_to_emulator_for_direct_and_legacy_calibration(self):
        direct=ik.demo_calibration().data
        legacy=copy.deepcopy(direct);legacy.pop('angle_convention')
        for name in mc.JOINTS:
            legacy['joints'][name]['points']=[[s,mc.Calibration.q_to_measured(name,q)]
                                            for s,q in direct['joints'][name]['points']]
        for data in (direct,legacy):
            app=recorded_app();app.cal=mc.Calibration(data);app.m=ik.model(gui.GEOMETRY,app.cal)
            app.inbox=queue.Queue();app.link=gui.SimulatedArm(app.inbox);app.root=SimpleNamespace(after=lambda *a:None)
            app.ready=False;app.initialized=False;app.busy=False;app.pending=None;app.plan=[];app.active=None
            app.preview=None;app.knobs=None;app.knob_baseline=None;app.mode='A';app.started=0
            app.sliders={i:Mock() for i in range(5)};app.readout=Mock();app.move_button=Mock();app.log=Mock()
            app.commanded=[90]*5
            app.poll();self.assertTrue(app.initialized)
            app.set_mode('I');app.poll();before=list(app.positions)
            app.entries=[SimpleNamespace(get=lambda value=str(x):value) for x in [100,100,50]]
            app.calculate();self.assertIsNotNone(app.preview,app.log.call_args_list)
            self.assertEqual(app.positions,before);self.assertFalse(app.busy)
            app.move();self.assertTrue(app.busy)
            for _ in range(2000):
                if app.link.timed is not None:app.link.timed[2]-=.05
                app.link.last_step=0;app.poll()
                if not app.busy:break
            self.assertFalse(app.busy)
            actual=ik.position(app.m,ik.servo_to_q(app.cal,app.positions))*1000
            self.assertLess(np.linalg.norm(actual-[100,100,50]),gui.MAX_MISS_MM)
            self.assertEqual(app.positions[0],before[0])

    def test_firmware_mapping_and_emulator_protocol_guards(self):
        source=(ROOT/'miniarm_controlC/miniarm_controlC/miniarm_controlC.ino').read_text()
        self.assertRegex(source,r'MOVE_ORDER\[5\]\s*=\s*\{\s*4,\s*3,\s*2,\s*1,\s*0\s*\}')
        self.assertIn('i == 0 ? 180 - value : value',source)  # claw inversion occurs once, in firmware
        inbox=queue.Queue();sim=gui.SimulatedArm(inbox);inbox.get()
        sim.send('J 40 90 90 90 90');self.assertEqual(inbox.get(),'ERR send INIT first')
        sim.send('CFG 40 90 10 170 10 170 0 180 0 180 23 7');inbox.get()
        self.assertEqual((sim.step_ms,sim.settle_ms),(23,7))
        sim.send('INIT 40 90 90 90 90');inbox.get();sim.send('J 40 95 95 95 95')
        self.assertIsNotNone(sim.seq)
        sim.send('INIT 40 90 90 90 90');self.assertIsNone(sim.seq);self.assertFalse(sim.move_active)
        sim.send('MODE X');self.assertEqual(inbox.get(),'OK');self.assertEqual(inbox.get(),'ERR MODE is A, K or I')


if __name__=='__main__':unittest.main()
