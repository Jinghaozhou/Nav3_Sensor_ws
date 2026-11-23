#!/usr/bin/env python3
import sys
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from scan_context_descriptor import encode, DEFAULT_CONFIG, config_hash, circular_distance, base_T_descriptor, estimate_stationary_up_odom


class DescriptorTests(unittest.TestCase):
    def setUp(self):
        self.p=np.random.default_rng(32).uniform([-3,-2,-1],[3,2,2],(2000,3))
        self.c=dict(DEFAULT_CONFIG,voxel_m=1e-6)

    def test_gravity_tilt_and_sensor_origin_invariance(self):
        a=encode(self.p,[0,0,1],[0,0,0],self.c)
        R=Rotation.from_euler('y',25,degrees=True).as_matrix();origin=np.array([.2,-.4,.5])
        b=encode(self.p@R.T+origin,R@[0,0,1],origin,self.c)
        np.testing.assert_allclose(a['descriptor'],b['descriptor'],atol=1e-5)
        np.testing.assert_allclose(b['base_T_descriptor'][:3,2],R@[0,0,1],atol=1e-12)

    def test_yaw_distance_and_shift_convention(self):
        a=encode(self.p,[0,0,1],[0,0,0],self.c)
        R=Rotation.from_euler('z',90,degrees=True).as_matrix()
        b=encode(self.p@R.T,[0,0,1],[0,0,0],self.c)
        score,shift=circular_distance(a['descriptor'],b['descriptor'])
        self.assertLess(score,1e-6);self.assertEqual(shift,45)

    def test_range_empty_and_invalid_up(self):
        p=np.vstack([self.p,[[100,0,0],[np.nan,0,0]]])
        r=encode(p,[0,0,1],[0,0,0],self.c)
        self.assertLessEqual(np.linalg.norm(r['points'],axis=1).max(),5.00001)
        for up in [[0,0,0],[np.nan,0,1]]:
            with self.assertRaises(ValueError):encode(p,up,[0,0,0],self.c)
        with self.assertRaises(ValueError):encode(np.zeros((3,3)),[0,0,1],[0,0,0],self.c)
        score,_=circular_distance(np.zeros((20,60)),np.zeros((20,60)))
        self.assertTrue(np.isinf(score))

    def test_vertical_heading_fallback_and_profile_identity(self):
        t,fallback=base_T_descriptor([1,0,0],[0,0,0]);self.assertTrue(fallback)
        self.assertAlmostEqual(np.linalg.det(t[:3,:3]),1.)
        self.assertNotEqual(config_hash(self.c),config_hash(dict(self.c,radius_m=4.)))

    def test_stationary_gravity_and_motion_rejection(self):
        a=np.tile([0,0,1.],(400,1));R=np.tile(np.eye(3),(400,1,1));w=np.zeros_like(a)
        np.testing.assert_allclose(estimate_stationary_up_odom(a,R,w),[0,0,1])
        with self.assertRaises(ValueError):estimate_stationary_up_odom(a,R,w+.2)


if __name__=='__main__':unittest.main()
