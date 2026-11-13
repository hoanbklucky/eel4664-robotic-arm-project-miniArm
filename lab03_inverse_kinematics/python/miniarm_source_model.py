"""Instructor-source geometry and angle convention; no hardware connection.

Original GUI uses millimetres/degrees and solves a 2x3 planar task.
Current simulation uses metres/radians, base 0..180 degrees, x right/y forward/z up.
Its XYZ axes are rotated relative to the original GUI's x forward/y left axes.
"""
import numpy as np
from kinematics import Model, position, jacobian, solve

GEOMETRY = dict(H=120., L1=56.5, L2=61.5, L3=85.)


def source_model():
    tool=np.eye(4); tool[0,3]=.085
    m=Model(np.array([[0,0,.120,0],[0,np.pi/2,0,0],
                           [.0565,0,0,0],[.0615,0,0,0]]), np.eye(4), tool,
                 np.deg2rad([[0,180],[0,180],[-160,160],[-160,160]]),
                 np.deg2rad([90,60,-60,-60]), example_only=True)
    # Provisional capsule radii: measure actual link/gripper/base envelopes.
    m.collision=dict(edges=np.array([[0,1],[2,3],[3,4],[4,5]]),
                     radii=np.array([.020,.006,.006,.006]),margin=.003,
                     pairs=np.array([[0,2],[0,3],[1,3]]),floor_edges=[1,2,3],
                     floor_z=0.,names=['base','upper arm','forearm','tool'])
    return m


def gui_to_math(q_deg, base_angle_deg=90.):
    return np.deg2rad([base_angle_deg, *q_deg])


def planar_position(q_deg):
    """Distance/height in mm, same mathematical convention as supplied GUI."""
    return 1000*position(source_model(), gui_to_math(q_deg,0.))[[0,2]]


def planar_jacobian(q_deg, method='partial'):
    """2x3, mm/rad, exactly the task/units of the instructor Jacobian."""
    J=jacobian(source_model(), gui_to_math(q_deg,0.), method)
    return 1000*J[[0,2],1:]


def xyz_from_gui_target(base_angle_deg, distance_mm, height_mm):
    b=np.deg2rad(base_angle_deg)
    return np.array([distance_mm*np.cos(b), distance_mm*np.sin(b), height_mm])/1000


def run_demo(target_mm=None, plot=False):
    m=source_model(); q_goal=np.deg2rad([110,70,-80,-40])
    target=position(m,q_goal) if target_mm is None else np.asarray(target_mm)/1000
    q,r=solve(m,target)
    print('Instructor dimensions; SIMULATION limits, no hardware connection')
    print('Target XYZ (mm):',target*1000)
    print('Base angle (deg):',np.rad2deg(q[0]))
    print('Arm/forearm/wrist q (deg):',np.rad2deg(q[1:]))
    print(r['status'],'residual (mm):',r['error']*1000)
    if plot:
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
        from kinematics import fk
        fig=plt.figure(); ax=fig.add_subplot(121,projection='3d')
        line,=ax.plot([],[],[],'o-',lw=2); ax.scatter(*target*1000,c='r',marker='x')
        ax.set(xlim=(-220,220),ylim=(-220,220),zlim=(-100,340),
               xlabel='x (mm)',ylabel='y (mm)',zlabel='z (mm)')
        ax.set_box_aspect([1,1,1])
        ax2=fig.add_subplot(122); ax2.semilogy(np.maximum(r['errors']*1000,1e-12),'o-')
        ax2.set(xlabel='Solver update (not time)',ylabel='Position error (mm)')
        def update(k):
            P=fk(m,r['history'][k])[1]*1000
            line.set_data_3d(*P.T); ax.set_title(f'Solver update {k}')
            return line,
        animation=FuncAnimation(fig,update,frames=len(r['history']),interval=120,repeat=True)
        plt.show()
        return animation
    return q,r


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description='Instructor-source miniArm simulation, XYZ mm')
    p.add_argument('--target-mm',nargs=3,type=float)
    p.add_argument('--plot',action='store_true')
    a=p.parse_args(); run_demo(a.target_mm,a.plot)
