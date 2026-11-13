"""Final-target-only motion, coordinated path checks and actual firmware host test."""
import copy
from pathlib import Path
import queue
import shutil
import subprocess
import tempfile
import unittest
from unittest.mock import patch, Mock
import numpy as np
import miniarm_controlC as gui
import miniarm_ik as ik
import miniarm_calibration as mc
from miniarm_version import APP_VERSION,DEVELOPER
from test_controls import controls_app


class TimedMotionTests(unittest.TestCase):
    def test_move_sends_one_final_target_not_solver_iterations(self):
        app=controls_app();app.poll();app.calculate()
        self.assertIsNotNone(app.preview);self.assertGreater(len(app.preview['result']['history']),10)
        self.assertEqual(len(app.preview['waypoints']),1)
        with patch.object(app.link,'send',wraps=app.link.send) as send:
            app.move()
        commands=[c.args[0] for c in send.call_args_list]
        self.assertEqual(len(commands),1);self.assertTrue(commands[0].startswith('T '))
        self.assertEqual(len(commands[0].split()),7)
        app.link.timed[2]-=30;app.poll();self.assertFalse(app.busy)

    def test_cubic_progress_all_axes_stop_and_fractional_resume(self):
        inbox=queue.Queue();sim=gui.SimulatedArm(inbox);inbox.get()
        sim.send('INIT 40 90 90 90 90');inbox.get()
        with patch.object(gui.time,'monotonic',return_value=100.):sim.send('T 2000 40 110 130 100 50')
        with patch.object(gui.time,'monotonic',return_value=100.5):sim.tick()
        np.testing.assert_allclose(sim.cur,[40,93.125,96.25,91.562,83.75],atol=.001)
        frozen=sim.cur.copy();sim.send('STOP')
        with patch.object(gui.time,'monotonic',return_value=1000.):sim.tick()
        self.assertEqual(sim.cur,frozen);self.assertTrue(sim.on)
        with patch.object(gui.time,'monotonic',return_value=1100.):sim.send('T 2000 40 90 90 90 90')
        with patch.object(gui.time,'monotonic',return_value=1101.):sim.tick()
        np.testing.assert_allclose(sim.cur,(np.array(frozen)+[40,90,90,90,90])/2,atol=.001)
        with patch.object(gui.time,'monotonic',return_value=1102.):sim.tick()
        self.assertEqual(sim.cur,[40,90,90,90,90]);self.assertIsNone(sim.timed)

    def test_coordinated_validation_splits_all_calibration_knots(self):
        cal=ik.demo_calibration();data=copy.deepcopy(cal.data)
        data['joints']['forearm']['points']=[[0,90],[40,60],[100,-10],[180,-90]]
        data['joints']['wrist']['points']=[[0,90],[30,50],[120,-50],[180,-90]]
        cal=mc.Calibration(data);m=ik.model(gui.GEOMETRY,cal);m.collision=None
        start=np.array([40,10,20,30,40.]);end=np.array([40,160,150,140,130.])
        fractions={0.,1.}
        for name in mc.JOINTS:
            i=mc.SERVO_INDEX[name]
            for raw,_ in cal.points[name]:
                f=(raw-start[i])/(end[i]-start[i])
                if 0<f<1:fractions.add(f)
        fractions=sorted(fractions)
        with patch.object(ik,'edge_clear',return_value=True) as check:
            ik.validate_coordinated_leg(m,cal,start,end)
        self.assertEqual(check.call_count,len(fractions)-1)
        for call,a,b in zip(check.call_args_list,fractions[:-1],fractions[1:]):
            np.testing.assert_allclose(call.args[1],ik.servo_to_q(cal,start+a*(end-start)))
            np.testing.assert_allclose(call.args[2],ik.servo_to_q(cal,start+b*(end-start)))

    def test_safe_endpoints_do_not_allow_colliding_direct_path(self):
        # Generic wider-range collision stress case, not miniArm operating limits.
        cal=ik.demo_calibration();data=copy.deepcopy(cal.data)
        for name in ('forearm','wrist'):data['joints'][name]['points']=[[0,180],[90,0],[180,-180]]
        cal=mc.Calibration(data);m=ik.model(gui.GEOMETRY,cal)
        m.limits[2:]=np.deg2rad([[-160,160],[-160,160]])
        m.collision['pairs']=np.empty((0,2),dtype=int);m.collision['floor_edges']=[3]
        a=np.deg2rad([90,30,-120,-160]);b=np.deg2rad([90,30,-120,160])
        start=ik.q_to_servo(cal,a,40);end=ik.q_to_servo(cal,b,40)
        self.assertGreater(ik.clearance(m,a)[0],0);self.assertGreater(ik.clearance(m,b)[0],0)
        with self.assertRaisesRegex(ValueError,'coordinated path'):
            ik.validate_coordinated_leg(m,cal,start,end)

    def test_duration_guard_and_old_firmware_capability(self):
        self.assertEqual(ik.motion_duration_ms([0]*5,[180]*5,.5),6000)
        for v in (0,np.nan,16):
            with self.assertRaises(ValueError):ik.motion_duration_ms([0]*5,[1]*5,v)
        app=controls_app(False);app.poll();app.initialize();app.inbox.put('OK');app.poll()
        app.preview={'prepared':True};app.mode='I';app.refresh_controls()
        self.assertEqual(app.move_button.options['state'],'disabled')
        with self.assertRaisesRegex(ValueError,'Upload'):app.launch([[40,90,90,90,90]],command='T',duration_ms=2000)
        app.inbox.put('FW 2.0.0 TIMED');app.inbox.put('READY');app.poll()
        self.assertTrue(app.supports_timed_motion())
        app.inbox.put('READY');app.poll();self.assertFalse(app.supports_timed_motion())

    def test_identity_matches_reported_firmware_version(self):
        self.assertEqual(DEVELOPER,'Ngo Lab');self.assertEqual(APP_VERSION,'2.0.0')
        sketch=(Path(__file__).parent/'miniarm_controlC/miniarm_controlC.ino').read_text()
        self.assertIn('FW '+APP_VERSION+' TIMED',sketch)

    @unittest.skipUnless(shutil.which('g++'),'Host C++ compiler unavailable')
    def test_actual_arduino_sketch_with_host_hardware_stubs(self):
        root=Path(__file__).parent
        with tempfile.TemporaryDirectory() as temp:
            binary=str(Path(temp)/'firmware_test')
            compile_result=subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(root/'qa/firmware_stubs'),
                                           str(root/'qa/firmware_timed_test.cpp'),'-o',binary],capture_output=True,text=True)
            self.assertEqual(compile_result.returncode,0,compile_result.stderr)
            result=subprocess.run([binary],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('fractional resume',result.stdout)


if __name__=='__main__':unittest.main()
