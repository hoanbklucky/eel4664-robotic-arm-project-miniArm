"""Runnable NumPy reference for the MATLAB Craig-MDH lab. SI units."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Model:
    mdh: np.ndarray
    base: np.ndarray
    tool: np.ndarray
    limits: np.ndarray
    q0: np.ndarray
    example_only: bool = True

    def validate(self):
        n = len(self.mdh)
        if self.mdh.shape != (n, 4) or n == 0 or not np.isfinite(self.mdh).all():
            raise ValueError('Invalid MDH table')
        if (self.limits.shape != (n, 2) or not np.isfinite(self.limits).all()
                or np.any(self.limits[:, 0] >= self.limits[:, 1])):
            raise ValueError('Invalid limits')
        self.check_q(self.q0, limits=True)
        for H in (self.base, self.tool):
            if (H.shape != (4, 4) or not np.isfinite(H).all()
                    or not np.allclose(H[3], [0, 0, 0, 1], atol=1e-12, rtol=0)
                    or not np.allclose(H[:3, :3].T @ H[:3, :3], np.eye(3), atol=1e-12, rtol=0)
                    or abs(np.linalg.det(H[:3, :3])-1) > 1e-12):
                raise ValueError('Invalid rigid transform')

    def check_q(self, q, limits=False):
        q = np.asarray(q, dtype=float)
        if q.shape != (len(self.mdh),) or not np.isfinite(q).all():
            raise ValueError('Invalid joint vector')
        if limits and np.any((q < self.limits[:, 0]) | (q > self.limits[:, 1])):
            raise ValueError('Seed outside limits')
        return q


def example_model():
    tool = np.eye(4); tool[0, 3] = .06
    return Model(np.array([[0, 0, .06, 0], [0, -np.pi/2, 0, 0],
                           [.1, 0, 0, 0], [.1, 0, 0, 0]]),
                 np.eye(4), tool, np.tile([-.8*np.pi, .8*np.pi], (4, 1)),
                 np.array([0., -.5, 1., -.3]))


def mdh(row, q):
    a, alpha, d, offset = row
    c, s = np.cos(q+offset), np.sin(q+offset)
    ca, sa = np.cos(alpha), np.sin(alpha)
    A = np.array([[c, -s, 0, a], [s*ca, c*ca, -sa, -d*sa],
                  [s*sa, c*sa, ca, d*ca], [0, 0, 0, 1.]])
    D = np.array([[-s, -c, 0, 0], [c*ca, -s*ca, 0, 0],
                  [c*sa, -s*sa, 0, 0], [0, 0, 0, 0.]])
    return A, D


def fk(m, q):
    q = m.check_q(q)
    T = m.base.copy(); points = [T[:3, 3].copy()]
    for row, qi in zip(m.mdh, q):
        T = T @ mdh(row, qi)[0]; points.append(T[:3, 3].copy())
    T = T @ m.tool; points.append(T[:3, 3].copy())
    return T, np.array(points)


def position(m, q):
    return fk(m, q)[0][:3, 3]


def jacobian(m, q, method='partial', h=1e-5):
    q = m.check_q(q); n = len(q); J = np.zeros((3, n))
    if method == 'finite':
        if not np.isfinite(h) or h <= 0:
            raise ValueError('h must be positive')
        for j in range(n):
            dq = np.zeros(n); dq[j] = h
            J[:, j] = (position(m, q+dq)-position(m, q-dq))/(2*h)
    elif method == 'partial':
        for j in range(n):
            dT = m.base.copy()
            for i, (row, qi) in enumerate(zip(m.mdh, q)):
                A, D = mdh(row, qi); dT = dT @ (D if i == j else A)
            J[:, j] = (dT @ m.tool)[:3, 3]
    else:
        raise ValueError('Use partial or finite')
    return J


def solve(m, target, q0=None, method='partial'):
    m.validate(); target = np.asarray(target, dtype=float)
    if target.shape != (3,) or not np.isfinite(target).all():
        raise ValueError('Target must be finite xyz')
    q = m.check_q(m.q0 if q0 is None else q0, limits=True).copy()
    history = [q.copy()]; errors = [float(np.linalg.norm(target-position(m, q)))]
    status = 'max_iterations'
    for _ in range(400):
        e = target-position(m, q)
        if np.linalg.norm(e) <= 1e-4:
            status = 'converged'; break
        J = jacobian(m, q, method)
        dq = J.T @ np.linalg.solve(J @ J.T + .01**2*np.eye(3), e)
        dq *= min(1., .10/max(np.linalg.norm(dq), np.finfo(float).eps))
        accepted = False
        for ls in range(16):
            trial = np.clip(q + .5**ls*dq, m.limits[:, 0], m.limits[:, 1])
            err = float(np.linalg.norm(target-position(m, trial)))
            if err < np.linalg.norm(e)-1e-12:
                q = trial; accepted = True; break
        if not accepted:
            status = 'stalled'; break
        history.append(q.copy()); errors.append(err)
    error = float(np.linalg.norm(target-position(m, q)))
    success = error <= 1e-4
    return q, dict(success=success, status='converged' if success else status,
                   error=error, history=np.array(history), errors=np.array(errors))


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Example-model position IK, metres')
    parser.add_argument('--target', type=float, nargs=3)
    parser.add_argument('--method', choices=['partial', 'finite'], default='partial')
    parser.add_argument('--plot', action='store_true')
    args = parser.parse_args()
    m = example_model()
    target = position(m, [.35, -.8, 1.2, -.2]) if args.target is None else np.array(args.target)
    q, r = solve(m, target, method=args.method)
    print('ILLUSTRATIVE MODEL ONLY; no hardware connection')
    print('target (m):', target, '\nq (deg):', np.rad2deg(q))
    print(r['status'], 'residual (m):', r['error'])
    if args.plot:
        import matplotlib.pyplot as plt
        fig = plt.figure(); ax = fig.add_subplot(121, projection='3d')
        for pose, label in [(m.q0, 'initial'), (q, 'final')]:
            _, P = fk(m, pose); ax.plot(*P.T, 'o-', label=label)
        ax.scatter(*target, marker='x', c='r', label='target')
        ax.set(xlabel='x (m)', ylabel='y (m)', zlabel='z (m)'); ax.legend()
        ax.set_box_aspect([1, 1, 1])
        ax2=fig.add_subplot(122); ax2.semilogy(np.maximum(r['errors'], 1e-16), 'o-')
        ax2.set(xlabel='Solver update', ylabel='Position error (m)'); plt.show()
