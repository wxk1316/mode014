"""Ground, ordinary road barriers, isolated returns, and building facades."""
import math
from pathlib import Path
import struct
import sys
import types
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa_real'),str(ROOT/'rda_dwa')]
from rda_dwa_real.perception_core import LidarPerceptionCore,PerceptionParams,point_cloud_xyz,bounding_box_xy


def ground(slope=(0.,0.)):
    x,y=np.meshgrid(np.linspace(-4,4,41),np.linspace(-4,4,41))
    z=-.81+slope[0]*x+slope[1]*y
    return np.column_stack((x.ravel(),y.ravel(),z.ravel()))


def barrier(cx,cy,height=.45,width=.3):
    x,y,z=np.meshgrid(np.linspace(cx-width/2,cx+width/2,5),np.linspace(cy-width/2,cy+width/2,5),np.linspace(-.81+.15,-.81+height,4))
    return np.column_stack((x.ravel(),y.ravel(),z.ravel()))


class PerceptionCoreTests(unittest.TestCase):
    def test_stride_endian_and_row_padding(self):
        for endian,big in [('<',False),('>',True)]:
            data=struct.pack(endian+'fff',1.,2.,3.)+b'abcd'+struct.pack(endian+'fff',4.,5.,6.)+b'abcd'
            msg=types.SimpleNamespace(height=2,width=1,point_step=12,row_step=16,is_bigendian=big,data=data,
                fields=[types.SimpleNamespace(name=n,offset=i*4,datatype=7,count=1) for i,n in enumerate(('x','y','z'))])
            np.testing.assert_allclose(point_cloud_xyz(msg),[[1,2,3],[4,5,6]])
        msg.data=b'bad'
        with self.assertRaises(ValueError):point_cloud_xyz(msg)

    def test_ground_removed_and_normal_barrier_retained(self):
        core=LidarPerceptionCore()
        detections,metrics=core.process(np.vstack((ground(),barrier(2.,0.))),100.)
        self.assertTrue(metrics['valid'])
        self.assertGreater(metrics['ground_removed_points'],500)
        self.assertEqual(len(detections),1)
        self.assertAlmostEqual(detections[0][0],2.,places=2)
        self.assertGreater(detections[0][3],.25)
        self.assertAlmostEqual(core.plane[2],-.36,places=2)

    def test_50cm_cone_with_28cm_base_and_partial_visible_face_is_retained(self):
        # A tapered visible face; only 85% of its width is sampled.
        cone=np.array([[2.-.14*(1.-h/.5), .85*y*(1.-h/.5), -.81+h]
                       for h in (.14,.25,.38,.5) for y in np.linspace(-.14,.14,7)])
        cloud=np.vstack((ground(),cone))
        old,_=LidarPerceptionCore(PerceptionParams(cluster_min_extent=.20)).process(cloud,100.)
        new,metrics=LidarPerceptionCore().process(cloud,100.)
        self.assertEqual(len(old),0)
        self.assertTrue(metrics['valid']);self.assertEqual(len(new),1)
        self.assertGreaterEqual(max(new[0][3:5]),.15);self.assertLess(max(new[0][3:5]),.20)

    def test_sloping_ground_and_ordinary_barrier(self):
        floor=ground((.04,-.02));obj=barrier(2.,0.)
        obj[:,2]+=.04*obj[:,0]-.02*obj[:,1]
        det,metrics=LidarPerceptionCore().process(np.vstack((floor,obj)),100.)
        self.assertTrue(metrics['valid']);self.assertEqual(len(det),1)
        self.assertGreater(metrics['ground_removed_points'],500)

    def test_two_isolated_returns_are_removed_without_hiding_barrier(self):
        noise=np.array([[-2,1,.3],[-2.01,1.01,.3]])
        det,metrics=LidarPerceptionCore().process(np.vstack((ground(),barrier(2.,0.),noise)),100.)
        self.assertEqual(len(det),1)
        self.assertGreaterEqual(metrics['weak_clusters_removed'],1)

    def test_neighboring_barriers_are_separate(self):
        det,_=LidarPerceptionCore().process(np.vstack((ground(),barrier(2.,-.4,width=.2),barrier(2.,.4,width=.2))),100.)
        self.assertEqual(len(det),2)

    def test_long_facade_is_tiled_and_all_observed_points_are_enclosed(self):
        x,z=np.meshgrid(np.linspace(-5.,5.,151),np.linspace(-.6,1.0,8))
        wall=np.column_stack((x.ravel(),np.full(x.size,2.),z.ravel()))
        det,metrics=LidarPerceptionCore().process(np.vstack((ground(),wall)),100.)
        self.assertGreater(metrics['structural_tiles'],1)
        self.assertTrue(all(max(d[3],d[4])<2. for d in det))
        for point in wall[::15]:
            enclosed=False
            for cx,cy,_,L,W,theta in det:
                dx,dy=point[0]-cx,point[1]-cy;c,s=math.cos(theta),math.sin(theta)
                enclosed|=abs(c*dx+s*dy)<=L/2+1e-8 and abs(-s*dx+c*dy)<=W/2+1e-8
            self.assertTrue(enclosed,point)

    def test_obb_uses_all_support_points(self):
        rng=np.random.default_rng(8);xy=rng.normal(size=(100,2))*.1
        cx,cy,L,W,t=bounding_box_xy(xy);c,s=math.cos(t),math.sin(t)
        x=(xy[:,0]-cx)*c+(xy[:,1]-cy)*s;y=-(xy[:,0]-cx)*s+(xy[:,1]-cy)*c
        self.assertLessEqual(max(abs(x)),L/2+1e-8);self.assertLessEqual(max(abs(y)),W/2+1e-8)

    def test_no_ground_evidence_does_not_fabricate_empty_free_scene(self):
        _,metrics=LidarPerceptionCore().process(barrier(2.,0.),100.)
        self.assertFalse(metrics['valid']);self.assertEqual(metrics['reason'],'ground_unavailable')

    def test_connected_building_corner_does_not_fill_courtyard(self):
        x,z=np.meshgrid(np.linspace(1.,5.,81),np.linspace(-.55,1.,6))
        horizontal=np.column_stack((x.ravel(),np.full(x.size,2.),z.ravel()))
        y,z=np.meshgrid(np.linspace(2.,5.,61),np.linspace(-.55,1.,6))
        vertical=np.column_stack((np.full(y.size,5.),y.ravel(),z.ravel()))
        det,metrics=LidarPerceptionCore().process(np.vstack((ground(),horizontal,vertical)),100.)
        self.assertGreater(metrics['structural_tiles'],1)
        for cx,cy,_,L,W,theta in det:
            dx,dy=3.-cx,3.5-cy;c,s=math.cos(theta),math.sin(theta)
            self.assertFalse(abs(c*dx+s*dy)<L/2 and abs(-s*dx+c*dy)<W/2)


if __name__=='__main__':unittest.main()
