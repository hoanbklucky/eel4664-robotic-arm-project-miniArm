"""Run: python -m unittest discover -s python -v (from lab03 folder)."""
import unittest
import numpy as np
from kinematics import example_model, mdh, position, jacobian, solve


class KinematicsTests(unittest.TestCase):
    def test_independent_closed_form_and_jacobian(self):
        m = example_model(); rng = np.random.default_rng(4664)
        for _ in range(100):
            q = rng.uniform(m.limits[:, 0], m.limits[:, 1])
            angles = np.cumsum(q[1:]); lengths = np.array([.1, .1, .06])
            rho = lengths @ np.cos(angles)
            expected = [rho*np.cos(q[0]), rho*np.sin(q[0]), .06-lengths@np.sin(angles)]
            np.testing.assert_allclose(position(m, q), expected, atol=1e-12, rtol=0)
            # Independent analytic derivative of closed-form yaw/pitch chain.
            J = np.zeros((3, 4)); J[:, 0] = [-rho*np.sin(q[0]), rho*np.cos(q[0]), 0]
            for j in range(1, 4):
                dr = -lengths[j-1:] @ np.sin(angles[j-1:])
                dz = -lengths[j-1:] @ np.cos(angles[j-1:])
                J[:, j] = [dr*np.cos(q[0]), dr*np.sin(q[0]), dz]
            np.testing.assert_allclose(jacobian(m, q), J, atol=1e-12, rtol=0)
            np.testing.assert_allclose(jacobian(m, q, 'finite'), J, atol=1e-8, rtol=0)
            dq = rng.normal(size=4)*1e-6
            self.assertLess(np.linalg.norm(position(m, q+dq)-position(m, q)-J@dq), 1e-10)

    def test_general_mdh_offsets_base_tool(self):
        m = example_model(); m.mdh[:, 3] = [.2, -.3, .4, .1]
        m.mdh[2, 1:3] = [.35, .02]
        m.base = mdh([.03, .2, .04, .3], 0)[0]
        m.tool = mdh([.05, -.4, .03, .2], 0)[0]
        m.validate()
        np.testing.assert_allclose(jacobian(m, m.q0), jacobian(m, m.q0, 'finite'), atol=1e-8, rtol=0)

    def test_reachable_limits_and_descent(self):
        m = example_model()
        for method in ('partial', 'finite'):
            for goal in ([.35, -.8, 1.2, -.2], [-.4, -.6, .9, .2], [.1, -.4, .8, -.1]):
                q, r = solve(m, position(m, goal), method=method)
                self.assertTrue(r['success'], r)
                self.assertLessEqual(r['error'], 1e-4)
                self.assertTrue(np.all(np.diff(r['errors']) <= 1e-12))
                self.assertTrue(np.all(r['history'] >= m.limits[:, 0]))
                self.assertTrue(np.all(r['history'] <= m.limits[:, 1]))

    def test_failure_zero_error_and_singular_seed(self):
        m = example_model()
        for method in ('partial', 'finite'):
            self.assertFalse(solve(m, [2., 0, 0], method=method)[1]['success'])
            r = solve(m, position(m, m.q0), method=method)[1]
            self.assertTrue(r['success']); self.assertEqual(len(r['history']), 1)
            q, r = solve(m, [.20, .04, .10], q0=np.zeros(4), method=method)
            self.assertTrue(np.isfinite(q).all())
            self.assertTrue(np.isfinite(r['errors']).all())

    def test_input_validation(self):
        m = example_model()
        for target in ([np.nan, 0, 0], [1, 2], [np.inf, 0, 0]):
            with self.assertRaises(ValueError): solve(m, target)
        with self.assertRaises(ValueError): solve(m, [0, 0, 0], q0=[9, 0, 0, 0])
        with self.assertRaises(ValueError): jacobian(m, m.q0, 'finite', h=0)
        m.tool[0, 0] = 2
        with self.assertRaises(ValueError): m.validate()


if __name__ == '__main__': unittest.main()
