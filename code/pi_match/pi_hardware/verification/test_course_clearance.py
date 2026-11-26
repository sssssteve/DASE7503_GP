from pathlib import Path
from types import SimpleNamespace as NS
import math
import sys
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from course_clearance import (
    check_corridor, grid_cell, keepout_cells, verify_drop, verify_end_standoff,
    rear_strip_blockage, rack_pose, verify_retreat, verify_travel,
)


class CourseClearanceTest(unittest.TestCase):
    def test_p1_backup_uses_real_hull_and_counts_p2_clearance_once(self):
        base=(1.569465288,.666532713,3.338365240)
        p2=(1.92,.40,math.pi)
        c=dict(base_size_m=(.28,.19),body_center_x_m=.03,rack_size_m=(.14,.38),qr_posts=[])
        grid=NS(info=NS(width=155,height=104,resolution=.02,
                origin=NS(position=NS(x=-.04,y=-.04),orientation=NS(w=1.,z=0.))),data=[0]*(155*104))
        mask=set(keepout_cells(grid,[],[p2],c['rack_size_m'],margin=.02))
        for i in mask:grid.data[i]=100
        registered=lambda p:grid_cell(grid,p) in mask
        # Isolate the old double-margin failure, then check the corrected wheel origin.
        self.assertIsNotNone(rear_strip_blockage(grid,base,-.08,c['base_size_m']))
        self.assertIsNone(rear_strip_blockage(grid,base,-.08,c['base_size_m'],registered))
        center=rack_pose(base,(.03,0.))
        verify_retreat(base,-.08,c,[p2])
        self.assertIsNone(rear_strip_blockage(grid,center,-.08,c['base_size_m'],registered))
        i=grid_cell(grid,rack_pose(base,(-.15,0.)))
        grid.data[i]=99
        self.assertIsNone(rear_strip_blockage(grid,center,-.08,c['base_size_m'],registered))
        for value in (100,-1):
            grid.data[i]=value
            self.assertIn('实扫区',rear_strip_blockage(grid,center,-.08,c['base_size_m'],lambda p:True))

    def setUp(self):
        self.c={'destination_bounds':(0,.3,.8,1.7),'rack_size_m':(.14,.38),
                'base_size_m':(.24,.20),'qr_posts':[(2.8,1.,.04),(.3,1.,.04)]}

    def test_escape_replay_allows_increasing_clearance_only(self):
        c=dict(base_size_m=(.28,.19),body_center_x_m=.03,rack_size_m=(.14,.38),qr_posts=[])
        base=(2.053,1.048,math.radians(163));rack=(1.81,1.37,math.pi)
        with self.assertRaisesRegex(ValueError,'登记货架'):
            verify_retreat(base,-.15,c,[rack])
        verify_retreat(base,-.15,c,[rack],allow_departure=True)
        for close in ((0.,0.,0.),(-.205,0.,0.)):
            with self.assertRaisesRegex(ValueError,'登记货架'):
                verify_retreat((0.,0.,0.),-.15,c,[close],allow_departure=True)

    def test_soft_margin_is_distinct_from_structure_and_preserves_unknown(self):
        from course_clearance import paint_keepout
        grid=NS(info=NS(width=100,height=100,resolution=.02,
                origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*10000)
        rack=(1.01,1.01,0.);size=(.14,.38)
        halo=grid_cell(grid,(1.11,1.01))
        unknown=grid_cell(grid,(1.11,1.03));grid.data[unknown]=-1
        wall=grid_cell(grid,(1.11,1.05));grid.data[wall]=100
        paint_keepout(grid,[],[rack],size,.02)
        self.assertEqual([grid.data[i] for i in (grid_cell(grid,rack),halo,unknown,wall)],[100,98,-1,100])

    def test_four_distinct_slots_can_fit_and_drop_cannot_touch(self):
        deposited=[]
        for x,y in ((.23,1.37),(.23,.60),(.60,1.37),(.60,.60)):
            pose=x,y,math.pi
            verify_drop(self.c,pose,deposited)
            verify_retreat(pose,-.25,self.c,deposited)
            deposited.append(pose)
        with self.assertRaisesRegex(ValueError,'与其他落架'):
            verify_drop(self.c,(.24,1.37,math.pi),deposited)
        with self.assertRaisesRegex(ValueError,'终点区'):
            verify_drop(self.c,(.60,.49,math.pi),deposited)
        with self.assertRaisesRegex(ValueError,'二维码立柱'):
            verify_drop(self.c,(.31,1.,math.pi),deposited)
        with self.assertRaisesRegex(ValueError,'登记货架'):
            verify_retreat((.22,1.37,math.pi),-.35,self.c,[deposited[2]])

    def test_occupied_map_includes_pillars_and_all_parked_racks(self):
        grid=NS(info=NS(width=155,height=104,resolution=.02,
                        origin=NS(position=NS(x=-.04,y=-.04),orientation=NS(w=1.,z=0.))))
        parked=[(.23,1.37,math.pi),(.60,.60,math.pi)]
        cells=set(keepout_cells(grid,self.c['qr_posts'],parked,self.c['rack_size_m']))
        self.assertTrue(all(grid_cell(grid,p) in cells for p in parked+self.c['qr_posts']))
        self.assertNotIn(grid_cell(grid,(1.4,1.)),cells)
        empty=set(keepout_cells(grid,self.c['qr_posts'],parked,self.c['rack_size_m'],margin=.02))
        self.assertLess(empty,cells)
        for point in parked+self.c['qr_posts']:
            self.assertIn(grid_cell(grid,point),empty)
        # 3 cm outside P1's right edge still intersects the two-centimetre
        # margin once cell coverage is included; 5 cm only hits the old margin.
        for x,masked in ((.33,True),(.35,False)):
            cell=grid_cell(grid,(x,1.37))
            self.assertIn(cell,cells)
            self.assertEqual(cell in empty,masked)

    def test_forward_and_reverse_guard_requires_clear_rays_and_refuses_obstruction(self):
        body=self.c['base_size_m']
        origin=(.19,0.)
        clear=[(-.8,-.16),(-.8,0.),(-.8,.16)]
        check_corridor(clear,origin,-.27,body)
        with self.assertRaisesRegex(RuntimeError,'缺少雷达覆盖'):
            check_corridor(clear[:1],origin,-.27,body)
        with self.assertRaisesRegex(RuntimeError,'定距路径内有'):
            check_corridor(clear+[(-.27,.06)],origin,-.27,body)
        check_corridor([(.8,-.16),(.8,0.),(.8,.16)],origin,.30,body)

    def test_loaded_sweep_stops_before_an_unseen_parked_rack(self):
        grid=NS(info=NS(width=150,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15000)
        parked=[(1.43,1.,0.)]
        verify_travel((1.,1.,0.),(.18,0.,0.),.15,grid,[],parked,self.c['rack_size_m'])
        with self.assertRaisesRegex(RuntimeError,'已登记的货架'):
            verify_travel((1.,1.,0.),(.18,0.,0.),.22,grid,[],parked,self.c['rack_size_m'])
        verify_travel((1.,1.,0.),(0.,.18,0.),.22,grid,[],parked,self.c['rack_size_m'])

    def test_loaded_box_does_not_project_through_static_wall(self):
        grid=NS(info=NS(width=150,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15000)
        for j in range(100):
            grid.data[j*150+70]=100
        with self.assertRaisesRegex(RuntimeError,'围墙'):
            verify_travel((1.05,1.,math.pi),(-.18,0.,0.),.22,grid,[],[],self.c['rack_size_m'])
        verify_travel((1.05,1.,math.pi),(.18,0.,0.),.22,grid,[],[],self.c['rack_size_m'])

    def test_loaded_sweep_uses_live_returns_in_an_empty_map(self):
        grid=NS(info=NS(width=150,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15000)
        raw=[(.34,.09),(.36,.09)]
        with self.assertRaisesRegex(RuntimeError,'实时雷达'):
            verify_travel((1.,1.,math.pi),(.18,0.,0.),.22,grid,[],[],self.c['rack_size_m'],scan=raw)
        verify_travel((1.,1.,math.pi),(0.,-.18,0.),.22,grid,[],[],self.c['rack_size_m'],scan=raw)

    def test_empty_base_scan_stop_and_swept_corner(self):
        grid=NS(info=NS(width=150,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15000)
        def check(hit,v=(0.,0.,0.),posts=()):
            verify_travel((1.,1.,0.),v,.17,grid,posts,[],(.14,.38),
                          scan=[hit],body_size=(.28,.19),scan_radius=.10)
        with self.assertRaisesRegex(RuntimeError,'实时雷达'):check((0.,.10))
        check((0.,.101))
        check((-.053,-.107))
        check((.155,.055))
        with self.assertRaisesRegex(RuntimeError,'车身扫过'):check((.155,.055),(0.,0.,-.8))
        with self.assertRaisesRegex(RuntimeError,'实时雷达'):check((.21,0.),(.18,0.,0.))
        with self.assertRaisesRegex(RuntimeError,'二维码立柱'):check((0.,.19),posts=[(1.,1.21,.01)])

    def test_rejects_an_end_goal_within_the_shift_envelope_or_goal_tolerance(self):
        end=(.30,1.,.03)
        shifted=(.30,1.,.11)
        goal=(.67,1.,0.)
        verify_end_standoff(goal,.20,[end])
        with self.assertRaisesRegex(ValueError,'END 停车扫码位'):
            verify_end_standoff(goal,.20,[shifted])
        verify_end_standoff((.76,1.,0.),.20,[shifted])

    def test_shifted_post_remains_a_keepout_without_any_scan_return(self):
        grid=NS(info=NS(width=150,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15000)
        shifted=[(.30,1.,.11)]
        self.assertIn(grid_cell(grid,(.40,1.)),keepout_cells(grid,shifted,[],self.c['rack_size_m']))
        with self.assertRaisesRegex(RuntimeError,'二维码立柱'):
            verify_travel((.72,1.,math.pi),(.17,0.,0.),.20,grid,shifted,[],self.c['rack_size_m'])


if __name__=='__main__':
    unittest.main()
