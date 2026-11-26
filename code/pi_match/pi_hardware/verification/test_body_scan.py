import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from start_region import combined_walls, filter_scan, mask_body, scan_components, scan_mask


class BodyScanTest(unittest.TestCase):
    def test_wheel_center_is_three_centimetres_behind_hull_center(self):
        xs=np.array([-.13,-.109,.155,.171])
        self.assertEqual(scan_mask(xs,np.zeros(4),(.28,.19),center_x=.03).tolist(),
                         [False,True,True,False])

    def test_compact_isolated_side_pair_but_not_three_beams_or_frontal_leg(self):
        def sample(degrees,distances):
            scan=NS(ranges=list(distances),angle_min=math.radians(degrees),
                    angle_increment=math.radians(.8),range_min=.03,range_max=12.)
            filter_scan(scan,[.10,0,0,0,0,0],[.28,.19],near_gap=.03,body_padding=.01)
            return scan.ranges
        self.assertTrue(all(math.isnan(r) for r in sample(212.94,[.244,.241])))
        self.assertEqual(sample(212.94,[.244,.241,.243]),[.244,.241,.243])
        self.assertEqual(sample(0.,[.244,.241]),[.244,.241])
        self.assertTrue(all(math.isnan(r) for r in sample(179.,[.275,.274])))
        self.assertEqual(sample(179.,[.275,.274,.276]),[.275,.274,.276])
        self.assertEqual(sample(179.,[.40,.401]),[.40,.401])
        self.assertEqual(sample(212.94,[.244,.258]),[.244,.258])

    def test_near_front_post_and_rear_body_at_same_range(self):
        scan=NS(ranges=[.08,.12,.08,.09],angle_min=0.,angle_increment=math.pi/2,
                range_min=.03,range_max=12.)
        mask_body(scan,[.10,0,0,0,0,0],[.28,.19])
        self.assertEqual(scan.ranges[:2],[.08,.12])
        self.assertTrue(all(math.isnan(r) for r in scan.ranges[2:]))

    def test_only_right_edge_patch_is_added(self):
        box=[.052,-.114,.068,-.093]
        u=np.array([.052,.068,.06,.0519,.0681,.06,.06,.15])
        v=np.array([-.114,-.096,-.10,-.10,-.10,-.1141,.10,0.])
        self.assertEqual(scan_mask(u,v,[.28,.19],box).tolist(),
                         [True,True,True,False,False,False,False,False])

    def test_remove_isolated_beams_preserve_leg_pairs_and_scan_seam(self):
        scan=NS(ranges=[math.inf]*360,angle_min=0.,angle_increment=math.tau/360,
                range_min=.03,range_max=12.)
        for i in (359,0,7,17,19,70):
            scan.ranges[i]=.5
        scan.ranges[71]=2.2
        filter_scan(scan,[.1,0,0,0,0,0],[.28,.19])
        for i in (359,0,17,19):self.assertEqual(scan.ranges[i],.5)
        for i in (7,70,71):self.assertTrue(math.isnan(scan.ranges[i]))
        self.assertEqual(scan.ranges[5],math.inf)

    def test_padding_stops_at_nose_and_rounds_rear_corners(self):
        u=np.array([0.,0.,-.067,-.146,-.148,.146])
        v=np.array([-.105,-.106,-.097,.101,.103,.101])
        self.assertEqual(scan_mask(u,v,[.28,.19],padding=.01).tolist(),
                         [True,False,True,True,False,False])

    def test_navigation_and_web_keep_leg_cluster_just_beyond_nose(self):
        # Three beams hit a flat face near the front-right contact reported on the car.
        for gap in (-.005,0.,.005,.01,.015):
            dx=.17+gap-.13
            angle=math.atan2(-.03,dx)
            step=math.radians(.8)
            scan=NS(ranges=[dx/math.cos(angle+i*step) for i in range(3)],
                    angle_min=angle,angle_increment=step,range_min=.03,range_max=12.)
            settings=dict(near_gap=.03,body_padding=.01,center_x=.03)
            args=(scan,[.13,0,0,0,0,0],[.28,.19])
            _,_,_,masked,isolated=scan_components(*args,**settings)
            self.assertEqual((~(masked|isolated)).tolist(),[gap>0]*3)
            filter_scan(*args,**settings)
            self.assertEqual([math.isfinite(r) for r in scan.ranges],[gap>0]*3)

    def test_two_fixed_sectors_keep_far_wall_on_same_bearings(self):
        windows=[[104.,112.,.095,.127],[-152.,-141.5,.158,.218]]
        for angle,near in ((106.,.11),(-145.,.187),(-150.,.193)):
            def sample(r):
                return NS(ranges=[r,r,r],angle_min=math.radians(angle),angle_increment=.006,
                          range_min=.03,range_max=12.)
            cluster=sample(near);wall=sample(.45)
            for msg in (cluster,wall):
                filter_scan(msg,[.1,0,0,0,0,0],[.28,.19],near_gap=.03,echo_windows=windows)
            self.assertTrue(all(math.isnan(v) for v in cluster.ranges))
            self.assertEqual(wall.ranges,[.45]*3)

    def test_wall_in_two_stopped_views(self):
        u=np.linspace(-.50,.50,31)
        v=np.full_like(u,.7)
        views=[((u,v),(0.,0.,0.)),((v,-u),(0.,0.,math.pi/2))]
        x,y=combined_walls(views,(0.,0.,0.))
        self.assertTrue(np.allclose(y,.7))
        self.assertGreaterEqual(np.ptp(x),.96)
