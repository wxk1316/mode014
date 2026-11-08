"""Raw connectivity, complete envelopes, and ten-centimetre campus boundaries."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'rda_dwa_real'),str(ROOT/'rda_dwa')]
from rda_dwa_real.perception_core import (LidarPerceptionCore,PerceptionParams,
    cell_index,voxel_components)
from rda_dwa_real.obstacle_fusion import Clusterer


def road(slope=(0.,0.),spacing=.2):
    axis=np.arange(-4.,4.001,spacing);x,y=np.meshgrid(axis,axis)
    return np.column_stack((x.ravel(),y.ravel(),(-.81+slope[0]*x+slope[1]*y).ravel()))


def returns(xy,height=.3,slope=(0.,0.)):
    xy=np.asarray(xy)
    return np.column_stack((xy,-.81+slope[0]*xy[:,0]+slope[1]*xy[:,1]+height))


def enclosed(xy,detections):
    result=np.zeros(len(xy),dtype=bool)
    for cx,cy,_,length,width,angle in detections:
        dx=xy[:,0]-cx;dy=xy[:,1]-cy;c,s=math.cos(angle),math.sin(angle)
        result|=(np.abs(c*dx+s*dy)<=length/2+1e-8)&(np.abs(-s*dx+c*dy)<=width/2+1e-8)
    return result


def grid_at(grid,point):
    x=int(math.floor((point[0]-grid['origin_x'])/grid['resolution']))
    y=int(math.floor((point[1]-grid['origin_y'])/grid['resolution']))
    return grid['data'][y*grid['width']+x]


def extrema_bridge():
    return np.array([[1.001,1.05],[1.099,1.01],[1.05,1.001],[1.01,1.099],[1.098,1.098],
                     [1.301,1.39],[1.399,1.35],[1.39,1.301],[1.35,1.399],[1.302,1.302]])


def u_shape():
    a=np.linspace(-1.,1.,41)
    return np.vstack((np.column_stack((np.ones(len(a)),a)),
                      np.column_stack((a,np.ones(len(a)))),
                      np.column_stack((a,-np.ones(len(a))))))


def graph_labels(points):
    cells,ids,counts=cell_index(np.floor(points/.1).astype(np.int32))
    starts=np.r_[0,np.cumsum(counts)[:-1]];ends=np.cumsum(counts)-1;indices=[]
    for axis in (0,1):
        order=np.lexsort((points[:,axis],ids));indices.extend((order[starts],order[ends]))
    representatives=np.stack([points[idx] for idx in indices],axis=1)
    labels,_=voxel_components(cells,representatives,.1,.3,points=points,point_cells=ids)
    return labels[ids]


class CampusPerceptionTests(unittest.TestCase):
    def process(self,xy,height=.3,params=None):
        return LidarPerceptionCore(params).process(np.vstack((road(),returns(xy,height))),100.)

    def test_non_extremal_bridge_keeps_complete_raw_component(self):
        xy=extrema_bridge();d,m=self.process(xy)
        self.assertTrue(m['valid']);self.assertEqual(m['cluster_count'],1)
        self.assertEqual(len(d),1);self.assertEqual(m['exact_voxel_queries'],1)
        self.assertTrue(enclosed(xy,d).all())

    def test_native_graph_matches_all_raw_points_including_ties_and_shuffles(self):
        rng=np.random.default_rng(71)
        for trial in range(8):
            points=np.round(rng.uniform(-1.5,1.5,(180,2)),2)
            points=np.vstack((points,extrema_bridge()))
            rng.shuffle(points)
            exact=Clusterer(.3,1,10.).cluster([tuple(p) for p in points])
            groups={p:i for i,g in enumerate(exact) for p in g}
            wanted=np.array([groups[tuple(p)] for p in points]);actual=graph_labels(points)
            self.assertTrue(np.array_equal(wanted[:,None]==wanted[None,:],actual[:,None]==actual[None,:]),trial)

    def test_distance_budget_failure_is_not_a_valid_partial_scene(self):
        _,m=self.process(extrema_bridge(),params=PerceptionParams(max_distance_tests=1))
        self.assertFalse(m['valid']);self.assertEqual(m['reason'],'cluster_distance_capacity')

    def test_sparse_structure_retains_every_accepted_parent_support(self):
        xy=np.column_stack((np.linspace(1.,5.,16),np.full(16,2.)))
        d,m=self.process(xy)
        self.assertEqual(m['cluster_count'],1);self.assertEqual(len(d),5)
        self.assertEqual(sum(s['points'] for s in m['detection_support']),16)
        self.assertTrue(enclosed(xy,d).all())

    def test_small_u_preserves_vehicle_cavity_and_all_wall_points(self):
        xy=u_shape();d,m=self.process(xy)
        self.assertGreater(len(d),1);self.assertTrue(enclosed(xy,d).all())
        # Entire physical car, not just its centre, lies in the empty cavity.
        x,y=np.meshgrid(np.linspace(-.6,.6,13),np.linspace(-.4,.4,9))
        self.assertFalse(enclosed(np.column_stack((x.ravel(),y.ravel())),d).any())
        self.assertTrue(any(s['cavity'] for s in m['detection_support']))

    def test_off_vehicle_small_corner_does_not_fill_empty_interior(self):
        a=np.linspace(1.,3.,41)
        xy=np.vstack((np.column_stack((a,np.ones(len(a)))),np.column_stack((np.ones(len(a)),a))))
        d,_=self.process(xy)
        self.assertTrue(enclosed(xy,d).all());self.assertFalse(enclosed(np.array([[2.,2.]]),d).any())

    def test_dense_platform_cannot_replace_known_ground(self):
        x,y=np.meshgrid(np.linspace(1.5,2.5,51),np.linspace(-.5,.5,51))
        xy=np.column_stack((x.ravel(),y.ravel()));core=LidarPerceptionCore()
        core.process(road(),100.)
        for i in range(4):
            d,m=core.process(np.vstack((road(),returns(xy))),100.1+i*.1)
            self.assertTrue(m['valid']);self.assertTrue(enclosed(xy,d).all())
            self.assertAlmostEqual(core.plane[2],-.36,places=6)
        # The first observed frame is also safe without an existing prior.
        d,m=LidarPerceptionCore().process(np.vstack((road(),returns(xy))),100.)
        self.assertTrue(m['valid']);self.assertTrue(enclosed(xy,d).all())

    def test_platform_only_cannot_initialize_road_and_reuse_expires(self):
        x,y=np.meshgrid(np.linspace(1.5,2.5,51),np.linspace(-.5,.5,51))
        cloud=returns(np.column_stack((x.ravel(),y.ravel())))
        _,m=LidarPerceptionCore().process(cloud,100.)
        self.assertFalse(m['valid'])
        core=LidarPerceptionCore();core.process(road(),100.)
        d,m=core.process(cloud,100.1)
        self.assertTrue(m['ground_reused']);self.assertTrue(d)
        _,m=core.process(cloud,101.1)
        self.assertFalse(m['valid']);self.assertNotIn('drivable_grid',m)

    def test_ten_cm_curb_and_dropoff_surfaces_are_blocked_on_both_sides(self):
        x,y=np.meshgrid(np.linspace(-4.,4.,81),np.linspace(-4.,4.,81))
        for sign in (-1.,1.):
            height=np.where(np.abs(y)<=.8,0.,sign*.1)
            cloud=np.column_stack((x.ravel(),y.ravel(),(-.81+height).ravel()))
            d,m=LidarPerceptionCore().process(cloud,100.)
            self.assertTrue(m['valid']);self.assertTrue(d)
            self.assertAlmostEqual(m['ground_plane'][2],-.36,places=5)
            g=m['drivable_grid'];self.assertEqual(grid_at(g,(2.,0.)),0)
            for pt in [(2.,2.),(2.,-2.),(0.,3.)]:self.assertEqual(grid_at(g,pt),100)

    def test_sparse_three_point_ten_cm_boundary_survives_noise_filter(self):
        xy=np.array([[2.,1.],[2.25,1.],[2.5,1.]])
        for height in (.1,-.1):
            d,m=self.process(xy,height)
            self.assertTrue(d);self.assertTrue(enclosed(xy,d).all())
            self.assertTrue(m['detection_support'][0]['boundary'])
        d,_=self.process(xy,.3)
        self.assertEqual(d,[])  # ordinary three-point returns remain weak

    def test_grade_and_measurement_noise_keep_road_distinct_from_curb(self):
        rng=np.random.default_rng(37);slope=(.04,-.02)
        floor=road(slope,spacing=.1);floor[:,2]+=rng.normal(0.,.008,len(floor))
        x,y=np.meshgrid(np.linspace(1.,4.,61),np.linspace(1.5,1.7,5))
        xy=np.column_stack((x.ravel(),y.ravel()));cloud=np.vstack((floor,returns(xy,.1,slope)))
        d,m=LidarPerceptionCore().process(cloud,100.)
        self.assertTrue(m['valid']);self.assertTrue(enclosed(xy,d).all())
        np.testing.assert_allclose(m['ground_plane'],[.04,-.02,-.36],atol=.008)

    def test_reused_ground_orientation_rotates_with_vehicle_yaw(self):
        core=LidarPerceptionCore();core.process(road((.04,-.02)),100.)
        xy=np.array([[2.+i*.04,y] for i in range(6) for y in (-.1,0.,.1)])
        d,m=core.process(returns(xy,.3,(-.02,-.04)),100.1,pose=(0.,0.,math.pi/2))
        self.assertTrue(m['ground_reused']);self.assertTrue(d)
        np.testing.assert_allclose(m['ground_plane'],[-.02,-.04,-.36],atol=1e-8)

    def test_grid_never_fills_unobserved_or_occluded_space(self):
        d,m=self.process(u_shape())
        grid=m['drivable_grid']
        self.assertEqual(grid_at(grid,(5.,5.)), -1)
        self.assertEqual(grid_at(grid,(0.,0.)), -1)  # self-filtered car interior
        self.assertTrue(set(grid['data'])<= {-1,0,100})
        self.assertEqual(grid['free_cells']+grid['blocked_cells']+grid['unknown_cells'],grid['width']*grid['height'])

    def test_blocked_return_overrides_road_support_in_same_cell(self):
        xy=np.array([[2.+i*.03,1.+j*.03] for i in range(5) for j in range(5)])
        _,m=self.process(xy,.1)
        self.assertEqual(grid_at(m['drivable_grid'],(2.04,1.04)),100)


if __name__=='__main__':unittest.main()
