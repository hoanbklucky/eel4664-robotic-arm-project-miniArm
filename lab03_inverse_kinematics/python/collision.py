"""Optional capsule approximation for the miniArm chain (not a CAD model)."""
import numpy as np
from kinematics import fk


def segment_distance(a,b,c,d):
    """Minimum Euclidean distance between two closed 3D line segments."""
    u=b-a; v=d-c; w=a-c
    aa=u@u; cc=v@v
    def point_segment(p,x,y):
        z=y-x; zz=z@z
        t=0. if zz<1e-24 else np.clip((p-x)@z/zz,0.,1.)
        return np.linalg.norm(p-(x+t*z))
    distances=[point_segment(a,c,d),point_segment(b,c,d),
               point_segment(c,a,b),point_segment(d,a,b)]
    normal=np.cross(u,v); det=normal@normal
    if det>1e-30 and aa>1e-24 and cc>1e-24:
        s=np.cross(c-a,v)@normal/det; t=np.cross(c-a,u)@normal/det
        if 0<=s<=1 and 0<=t<=1:
            distances.append(np.linalg.norm(w+s*u-t*v))
    return float(min(distances))


def validate_collision(m):
    c=m.collision
    if c is None: return
    n=len(m.mdh)+2
    edges=np.asarray(c['edges']); radii=np.asarray(c['radii']); pairs=np.asarray(c['pairs'])
    if (edges.ndim!=2 or edges.shape[1]!=2 or not np.issubdtype(edges.dtype,np.integer)
            or np.any(edges<0) or np.any(edges>=n)
            or radii.shape!=(len(edges),) or not np.isfinite(radii).all() or np.any(radii<=0)
            or pairs.ndim!=2 or pairs.shape[1]!=2 or not np.issubdtype(pairs.dtype,np.integer)
            or np.any(pairs<0) or np.any(pairs>=len(edges))
            or not np.isfinite(c['margin']) or c['margin']<0
            or not np.isfinite(c['floor_z'])):
        raise ValueError('Invalid collision configuration')
    ids=np.asarray(c['floor_edges'])
    if (not np.issubdtype(ids.dtype,np.integer) or np.any(ids<0) or np.any(ids>=len(edges))
            or len(c['names'])!=len(edges) or (len(pairs)==0 and len(ids)==0)):
        raise ValueError('Invalid floor edges')


def clearance(m,q):
    """Signed minimum clearance in metres; positive means separated."""
    if m.collision is None: return np.inf,'disabled'
    c=m.collision; P=fk(m,q)[1]; values=[]; labels=[]
    for i,j in c['pairs']:
        a,b=P[c['edges'][i]]; x,y=P[c['edges'][j]]
        values.append(segment_distance(a,b,x,y)-c['radii'][i]-c['radii'][j]-c['margin'])
        labels.append(f"{c['names'][i]} / {c['names'][j]}")
    for i in c['floor_edges']:
        values.append(np.min(P[c['edges'][i],2])-c['floor_z']-c['radii'][i]-c['margin'])
        labels.append(f"{c['names'][i]} / table")
    k=int(np.argmin(values)); return float(values[k]),labels[k]


def edge_clear(m,start,end,depth=0):
    """Conservative continuous check of straight JOINT interpolation.

    Bound point travel by chain_length*sum(abs(dq)), then bound clearance
    decrease by twice that travel. Subdivide if not certified; fail closed
    at depth 12. This applies only to configured capsules/pairs and floor.
    """
    if m.collision is None: return True
    c0,_=clearance(m,start); c1,_=clearance(m,end)
    if min(c0,c1)<=0: return False
    reach=np.sum(np.abs(m.mdh[:,0]))+np.sum(np.abs(m.mdh[:,2]))+np.linalg.norm(m.tool[:3,3])
    bound=2*reach*np.sum(np.abs(end-start))
    if max(c0,c1)>bound+1e-12: return True
    if depth>=12: return False
    mid=(start+end)/2
    return edge_clear(m,start,mid,depth+1) and edge_clear(m,mid,end,depth+1)
