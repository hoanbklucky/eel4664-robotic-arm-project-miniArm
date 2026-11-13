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
        # Measured raw limits may narrow, but never enlarge, the model limits.
        calibrated = np.deg2rad(bounds)
        m.limits = np.column_stack((np.maximum(m.limits[:,0],calibrated[:,0]),
                                    np.minimum(m.limits[:,1],calibrated[:,1])))
        if np.any(m.limits[:,0] >= m.limits[:,1]):
            raise ValueError('Calibration has no usable overlap with model joint limits; check angle references')
        m.q0 = np.clip(m.q0,m.limits[:,0],m.limits[:,1])
    m.validate()
    return m

def servo_to_q(cal, pose):
    """S1..S5 -> [base,shoulder,elbow,wrist], radians."""
    return np.deg2rad([cal.servo_to_q(name,pose[mc.SERVO_INDEX[name]])
                      for name in JOINTS])

def q_to_servo(cal,q,claw):
    deg=np.rad2deg(q)
    pose=[float(claw),0,0,0,0]
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
            delta=end[index]-cur[index]
            cur[index]=end[index] if abs(delta)<=1 else cur[index]+np.sign(delta)
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

def validate_coordinated_leg(m,cal,start,end):
    """Check the exact common-progress raw-command path used by firmware T.

    Cubic easing changes timing, not this path. Split at every calibration knot
    crossed by ANY channel. Between knots q(raw) is affine, so edge_clear checks
    continuous joint interpolation exactly for the configured collision model.
    """
    start=np.asarray(start,dtype=float);end=np.asarray(end,dtype=float)
    if start.shape!=(5,) or end.shape!=(5,) or not np.isfinite([start,end]).all():
        raise ValueError('Servo poses must contain five finite raw values')
    for pose in (start,end):
        if any(not lo<=s<=hi for s,(lo,hi) in zip(pose,cal.all_limits())):
            raise ValueError('Servo pose outside calibrated raw limits')
        m.check_q(servo_to_q(cal,pose),limits=True)
    fractions={0.,1.};delta=end-start
    for name in JOINTS:
        index=mc.SERVO_INDEX[name]
        if delta[index]:
            for raw,_ in cal.points[name]:
                f=(raw-start[index])/delta[index]
                if 0<f<1:fractions.add(float(f))
    previous=servo_to_q(cal,start)
    if clearance(m,previous)[0]<=0:
        raise ValueError('Starting pose collides with configured model')
    for f in sorted(fractions)[1:]:
        current=servo_to_q(cal,start+f*delta)
        m.check_q(current,limits=True)
        if not edge_clear(m,previous,current):
            raise ValueError('Direct coordinated path violates configured clearance; choose an intermediate XYZ target')
        previous=current


def motion_duration_ms(start,end,requested_seconds=2.):
    """Cubic peak speed <=45 raw units/s; 0.5..15 s user duration is a minimum."""
    if not np.isfinite(requested_seconds) or not .5<=requested_seconds<=15:
        raise ValueError('Move time must be between 0.5 and 15 seconds')
    delta=float(np.max(np.abs(np.asarray(end)-np.asarray(start))))
    if not np.isfinite(delta):raise ValueError('Invalid servo target')
    return int(np.ceil(max(requested_seconds*1000,1500*delta/45)))


def plan_motion(m,cal,start,target_mm,result,max_miss_mm=10):
    """Send only the final IK pose; validate the firmware's coordinated path."""
    if not result['success']:
        raise ValueError('IK failed: '+result['status'])
    if not np.allclose(servo_to_q(cal,start),result['history'][0],atol=1e-10,rtol=0):
        raise ValueError('Arm moved since calculation; calculate again')
    final=q_to_servo(cal,result['history'][-1],start[0])
    validate_coordinated_leg(m,cal,start,final)
    final_q=servo_to_q(cal,final)
    miss=float(np.linalg.norm(position(m,final_q)*1000-target_mm))
    if not np.isfinite(miss) or miss>max_miss_mm:
        raise ValueError(f'After rounding: error {miss:.2f} mm exceeds {max_miss_mm:g} mm')
    return [final],miss

def demo_calibration():
    """Invented mapping for the OFFLINE emulator, never a hardware calibration.

    q_base=s, q_shoulder=180-s, q_elbow=q_wrist=90-s (degrees).
    Raw 0..180 covers base/shoulder 0..180 and elbow/wrist +/-90.
    Real servo offsets/directions must come from measurements, not this table.
    """
    data=dict(angle_convention='joint',joints={
        'base':dict(points=[[0,0],[90,90],[180,180]],min=0,max=180),
        'arm':dict(points=[[0,180],[90,90],[180,0]],min=0,max=180),
        'forearm':dict(points=[[0,90],[90,0],[180,-90]],min=0,max=180),
        'wrist':dict(points=[[0,90],[90,0],[180,-90]],min=0,max=180)},
        claw=dict(open=40,closed=90,min=40,max=90))
    return mc.Calibration(data)
