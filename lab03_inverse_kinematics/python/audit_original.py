"""Audit a supplied miniarm_ik.py without changing it or connecting hardware.

Run: python python/audit_original.py path/to/miniarm_ik.py
Only load trusted source: importing a Python file executes its top-level code.
"""
import argparse
import base64
import json
from pathlib import Path
import sys
import types
import numpy as np


def audit(source):
    original=types.ModuleType('instructor_original')
    exec(compile(source,'<supplied miniarm_ik.py>','exec'),original.__dict__)
    g=dict(H=120.,L1=56.5,L2=61.5,L3=85.)
    rng=np.random.default_rng(4664); worst_j=worst_fk=0.
    def p(q):
        f=original.forward_kinematics(q,g)
        return np.array([f['distance'],f['height']])
    for _ in range(100):
        q=rng.uniform([10,-120,-120],[160,100,100]); a=np.cumsum(np.deg2rad(q))
        L=np.array([g['L1'],g['L2'],g['L3']])
        expected=[L@np.cos(a),g['H']+L@np.sin(a)]
        worst_fk=max(worst_fk,float(np.linalg.norm(p(q)-expected)))
        J=np.asarray(original.jacobian(q,g)); N=np.zeros((2,3)); h=1e-5
        for j in range(3):
            dq=np.zeros(3); dq[j]=np.rad2deg(h)
            N[:,j]=(p(q+dq)-p(q-dq))/(2*h)
        worst_j=max(worst_j,float(np.linalg.norm(J-N)))
    assert worst_fk<1e-9 and worst_j<1e-6
    results=[]
    for goal in ([70,-80,-40],[50,-70,-50],[110,-100,-30]):
        target=p(goal); answer=original.inverse_kinematics(*target,g,[60,-60,-60])
        assert answer is not None
        q,it,reported=answer; actual=float(np.linalg.norm(p(q)-target))
        assert actual<.1 and abs(actual-reported)<1e-9
        results.append(dict(target_mm=target.tolist(),iterations=it,error_mm=actual))
    assert original.inverse_kinematics(1000,120,g,[60,-60,-60]) is None
    report=dict(random_poses=100,max_fk_error_mm=worst_fk,
                max_jacobian_difference_mm_per_rad=worst_j,
                reachable_solutions=results,unreachable_returned_none=True)
    print(json.dumps(report,indent=2))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(); p.add_argument('source',nargs='?')
    p.add_argument('--stdin-base64',action='store_true',help=argparse.SUPPRESS)
    a=p.parse_args()
    if a.stdin_base64: text=base64.b64decode(sys.stdin.read()).decode('utf-8-sig')
    elif a.source: text=Path(a.source).read_text(encoding='utf-8-sig')
    else: p.error('Supply your miniarm_ik.py path')
    audit(text)
