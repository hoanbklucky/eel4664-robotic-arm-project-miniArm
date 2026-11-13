import unittest
import numpy as np
from collision import segment_distance, clearance, edge_clear
from miniarm_source_model import source_model
from kinematics import position, solve


class CollisionTests(unittest.TestCase):
    def test_segment_distance_cases(self):
        for a,b,c,d,expected in [
            ([0,0,0],[1,0,0],[.5,-1,0],[.5,1,0],0),
            ([0,0,0],[1,0,0],[0,-1e-8,0],[1,1e-8,0],0),
            ([0,0,0],[1,0,0],[0,1,0],[1,1,0],1),
            ([0,0,0],[1,0,0],[.5,-1,1],[.5,1,1],1),
            ([0,0,0],[1,0,0],[2,0,0],[3,0,0],1),
            ([0,0,0],[0,0,0],[1,-1,0],[1,1,0],1),
            ([0,0,0],[0,0,0],[0,0,0],[0,0,0],0)]:
            A,B,C,D=map(lambda x:np.array(x,dtype=float),(a,b,c,d))
            self.assertAlmostEqual(segment_distance(A,B,C,D),expected)
            self.assertAlmostEqual(segment_distance(C,D,A,B),expected)

    def test_base_intersection_rejected_even_at_zero_position_error(self):
        m=source_model(); q=np.deg2rad([90,100,-120,-120])
        distance,pair=clearance(m,q)
        self.assertLess(distance,0); self.assertEqual(pair,'base / tool')
        _,r=solve(m,position(m,q),q0=q)
        self.assertFalse(r['success']); self.assertEqual(r['status'],'seed_in_collision')

    def test_regression_old_solver_converges_into_base(self):
        m=source_model(); target=position(m,np.deg2rad([90,120,-100,-120]))
        old=source_model(); old.collision=None
        old_q,old_r=solve(old,target)
        self.assertTrue(old_r['success']); self.assertLess(clearance(m,old_q)[0],0)
        q,r=solve(m,target)
        self.assertFalse(r['success']); self.assertGreater(clearance(m,q)[0],0)
        for a,b in zip(r['history'][:-1],r['history'][1:]):
            self.assertTrue(edge_clear(m,a,b))

    def test_clear_endpoints_with_table_collision_between(self):
        m=source_model()
        # Isolate table so this regression specifically tests interpolation.
        m.collision['pairs']=np.empty((0,2),dtype=int)
        m.collision['floor_edges']=[3]
        a=np.deg2rad([90,30,-120,-160]); b=np.deg2rad([90,30,-120,160])
        self.assertGreater(clearance(m,a)[0],0); self.assertGreater(clearance(m,b)[0],0)
        self.assertLess(clearance(m,(a+b)/2)[0],0)
        self.assertFalse(edge_clear(m,a,b))

    def test_safe_goal_paths_and_yaw_invariance(self):
        m=source_model(); target=position(m,np.deg2rad([110,70,-80,-40]))
        for method in ('partial','finite'):
            q,r=solve(m,target,method=method)
            self.assertTrue(r['success']); self.assertGreater(r['clearance'],0)
            for a,b in zip(r['history'][:-1],r['history'][1:]):
                self.assertTrue(edge_clear(m,a,b))
        a=m.q0.copy(); b=a.copy(); b[0]=1.
        self.assertAlmostEqual(clearance(m,a)[0],clearance(m,b)[0])

    def test_invalid_collision_configuration(self):
        m=source_model();m.collision['radii'][0]=-1
        with self.assertRaises(ValueError): m.validate()


if __name__=='__main__': unittest.main()
