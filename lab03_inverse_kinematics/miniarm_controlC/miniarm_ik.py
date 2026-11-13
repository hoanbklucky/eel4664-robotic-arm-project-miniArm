"""3D GUI adapter; mm/degrees at display, metres/radians inside the solver.

Shares ../python/kinematics.py with MATLAB reference checks. No second solver.
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from kinematics import fk, position, jacobian as core_jacobian, solve
from collision import clearance, edge_clear
from miniarm_source_model import source_model
import miniarm_calibration as mc

JOINTS = mc.JOINTS  # mathematical q order, NOT S1..S5 transmission order

def model(geometry, calibration=None):
    """Match miniarm_model.m; replace dimensions and limits with measurements."""
    m = source_model()
    m.mdh[0,2] = geometry['H']/1000
    m.mdh[2,0] = geometry['L1']/1000
    m.mdh[3,0] = geometry['L2']/1000
    m.tool[0,3] = geometry['L3']/1000
    if calibration:
        bounds = []
        for name in JOINTS:
            lo, hi = calibration.limits(mc.SERVO_INDEX[name])
            values = [calibration.servo_to_q(name,s) for s in (lo,hi)]
            bounds.append(sorted(values))
        m.limits = np.deg2rad(bounds)
        m.q0 = np.clip(m.q0,m.limits[:,0],m.limits[:,1])
    m.validate()
    return m

def servo_to_q(cal, pose):
    """S1..S5 -> [base,shoulder,elbow,wrist], radians."""
    return np.deg2rad([cal.servo_to_q(name,pose[mc.SERVO_INDEX[name]])
                      for name in JOINTS])

def q_to_servo(cal,q,claw):
    deg=np.rad2deg(q)
    pose=[int(claw),0,0,0,0]
    for name,value in zip(JOINTS,deg):
        pose[mc.SERVO_INDEX[name]]=cal.q_to_servo(name,value)
    if any(not lo<=s<=hi for s,(lo,hi) in zip(pose,cal.all_limits())):
        raise ValueError('Rounded solution exceeds calibrated servo limits')
    return pose

def forward_kinematics(q_deg,geometry):
    T,P=fk(model(geometry),np.deg2rad(q_deg))
    return dict(T=T,tool=T[:3,3]*1000,joints=P*1000)

def jacobian(q_deg,geometry,method='partial'):
    """3x4 mm/rad: column j = [dx/dqj,dy/dqj,dz/dqj].

    'finite': perturb only joint j, divide XYZ change by radians changed.
    'partial': differentiate that joint's transform in the FK product.
    """
    return core_jacobian(model(geometry),np.deg2rad(q_deg),method)*1000

def inverse_kinematics(target_mm,m,q_start,method='partial'):
    """Same DLS, backtracking, limits and collision checks as MiniArmIK.solve.

    Target XYZ is mm; q_start and returned q are radians. Inspect success/status.
    """
    return solve(m,np.asarray(target_mm,dtype=float)/1000,q_start,method)

def sequential_states(start,end):
    """Exact J setpoints, one raw unit/tick: base,arm,forearm,wrist,claw."""
    cur=list(start)
    for index in (4,3,2,1,0):
        while cur[index]!=end[index]:
            cur[index]+=1 if end[index]>cur[index] else -1
            yield cur.copy()

def validate_servo_leg(m,cal,start,end):
    """Check each Arduino J tick, including continuous single-joint edges.

    Split at calibration knots to preserve piecewise-linear angle conversion.
    Capsules do not model claw opening, held objects, camera or surroundings.
    """
    for pose in (start,end):
        if any(not lo<=v<=hi for v,(lo,hi) in zip(pose,cal.all_limits())):
            raise ValueError('Servo state outside calibrated limits')
    previous=list(start)
    if clearance(m,servo_to_q(cal,previous))[0]<=0:
        raise ValueError('Starting pose collides with configured model')
    for state in sequential_states(start,end):
        index=next(i for i in range(5) if previous[i]!=state[i])
        if index:
            name=next(n for n,i in mc.SERVO_INDEX.items() if i==index)
            low,high=sorted((previous[index],state[index]))
            knots=[s for s,_ in cal.points[name] if low<s<high]
            samples=[*sorted(knots,reverse=state[index]<previous[index]),state[index]]
            qa=servo_to_q(cal,previous)
            for raw in samples:
                partial=list(previous); partial[index]=raw
                qb=servo_to_q(cal,partial)
                if not edge_clear(m,qa,qb):
                    raise ValueError('Sequential servo path violates configured clearance')
                qa=qb
        previous=state

def plan_motion(m,cal,start,target_mm,result,max_miss_mm=10):
    """Round accepted IK poses, check J execution path, preserve claw opening."""
    if not result['success']:
        raise ValueError('IK failed: '+result['status'])
    if not np.allclose(servo_to_q(cal,start),result['history'][0],atol=1e-10):
        raise ValueError('Arm moved since calculation; calculate again')
    waypoints=[]; previous=list(start)
    for q in result['history'][1:]:
        pose=q_to_servo(cal,q,start[0])
        if pose!=previous:
            validate_servo_leg(m,cal,previous,pose)
            waypoints.append(pose); previous=pose
    final_q=servo_to_q(cal,previous)
    m.check_q(final_q,limits=True)
    miss=float(np.linalg.norm(position(m,final_q)*1000-target_mm))
    if not np.isfinite(miss) or miss>max_miss_mm:
        raise ValueError(f'After rounding: error {miss:.2f} mm exceeds {max_miss_mm:g} mm')
    return waypoints,miss

def demo_calibration():
    """Synthetic affine mappings whose joint limits equal miniarm_model.m.

    This is not a claim about physical servo travel. The mapping is never
    saved as a real calibration. Pitch q=180-2*s spans +/-160 degrees over
    raw 10..170; base q=s and shoulder q=180-s span 0..180 over raw 0..180.
    """
    data=dict(angle_convention='joint',joints={
        'base':dict(points=[[0,0],[90,90],[180,180]],min=0,max=180),
        'arm':dict(points=[[0,180],[90,90],[180,0]],min=0,max=180),
        'forearm':dict(points=[[10,160],[90,0],[170,-160]],min=10,max=170),
        'wrist':dict(points=[[10,160],[90,0],[170,-160]],min=10,max=170)},
        claw=dict(open=40,closed=90,min=40,max=90))
    return mc.Calibration(data)
