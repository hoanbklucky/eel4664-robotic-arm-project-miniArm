import unittest
from pathlib import Path
import re
import numpy as np
from kinematics import position, jacobian, solve
from miniarm_source_model import (source_model, gui_to_math, planar_position,
                                 planar_jacobian, xyz_from_gui_target)


class SourceModelTests(unittest.TestCase):
    def test_matlab_configuration_matches_reference(self):
        text=(Path(__file__).resolve().parents[1]/'matlab'/'miniarm_model.m').read_text()
        def matrix(field):
            match=re.search(r'm\.'+field+r'\s*=\s*(?:deg2rad\()?\[(.*?)\]',text,re.S)
            self.assertIsNotNone(match)
            def value(token): return np.pi/2 if token=='pi/2' else float(token)
            return np.array([[value(v) for v in row.split()]
                             for row in match.group(1).split(';')])
        m=source_model()
        np.testing.assert_allclose(matrix('mdh'),m.mdh)
        np.testing.assert_allclose(np.deg2rad(matrix('limits')),m.limits)
        np.testing.assert_allclose(np.deg2rad(matrix('q0')).ravel(),m.q0)
        np.testing.assert_allclose(matrix('demoGoal').ravel(),[110,70,-80,-40])
        self.assertIn('m.tool(1,4) = .085',text)

    def test_reference_poses_and_frame_sign(self):
        m=source_model()
        for q,base,expected in [([90,0,0],90,[0,0,323]),
                                 ([0,0,0],0,[203,0,120]),
                                 ([0,0,0],90,[0,203,120]),
                                 ([0,0,0],180,[-203,0,120])]:
            np.testing.assert_allclose(position(m,gui_to_math(q,base))*1000,expected,atol=1e-10)

    def test_original_planar_formula_against_mdh_and_perturbation(self):
        m=source_model(); rng=np.random.default_rng(4664); L=np.array([56.5,61.5,85.])
        for _ in range(100):
            q=rng.uniform([10,-120,-120],[160,100,100]); a=np.cumsum(np.deg2rad(q))
            expected=np.array([L@np.cos(a),120+L@np.sin(a)])
            np.testing.assert_allclose(planar_position(q),expected,atol=1e-10)
            # Same symbolic formula as the supplied jacobian() (independent of MDH product).
            J=np.array([[-L[j:]@np.sin(a[j:]) for j in range(3)],
                         [L[j:]@np.cos(a[j:]) for j in range(3)]])
            np.testing.assert_allclose(planar_jacobian(q),J,atol=1e-10)
            np.testing.assert_allclose(planar_jacobian(q,'finite'),J,atol=1e-7)
            # Perturb degrees, divide by RADIANS: matches original API convention.
            h=1e-5; N=np.zeros((2,3))
            for j in range(3):
                dq=np.zeros(3); dq[j]=np.rad2deg(h)
                N[:,j]=(planar_position(q+dq)-planar_position(q-dq))/(2*h)
            np.testing.assert_allclose(N,J,atol=1e-7)

    def test_full_xyz_and_reachable_goals(self):
        m=source_model()
        for goal in ([110,70,-80,-40],[60,50,-70,-50],[90,100,-80,-30]):
            target=position(m,np.deg2rad(goal))
            for method in ('partial','finite'):
                _,r=solve(m,target,method=method)
                self.assertTrue(r['success'],r['status'])
        for b in [0,45,90,135,180]:
            target=xyz_from_gui_target(b,100,150)
            q=gui_to_math([60,-50,-30],b)
            np.testing.assert_allclose(jacobian(m,q),jacobian(m,q,'finite'),atol=1e-8)
            self.assertAlmostEqual(np.linalg.norm(target[:2]),.1)
        self.assertFalse(solve(m,[1,0,.12])[1]['success'])


if __name__=='__main__': unittest.main()
