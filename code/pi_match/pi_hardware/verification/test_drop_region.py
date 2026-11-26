import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from course_clearance import grid_cell,rack_pose
from drop_region import candidates,obstacle_grid,verify_region_pose


class DropRegionTest(unittest.TestCase):
    def setUp(self):
        self.grid=NS(info=NS(width=151,height=100,resolution=.02,
                  origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*15100)
        self.c=dict(destination_bounds=[0,.30,.80,1.70],rack_size_m=[.14,.38],
                    base_size_m=[.24,.20],radius_loaded_m=.205,radius_empty_m=.15,
                    load_center_m=(.015,0.),retreat_m=.27,qr_posts=[(.08,1.,.025)])
        self.reserve=[((.82,.99,0.),.15)]

    def choose(self,parked):
        return next(candidates(self.c,parked,obstacle_grid(self.grid),self.reserve,lambda:None),None)

    def test_four_successive_auto_drops_use_the_region(self):
        parked=[]
        for _ in range(4):
            p=self.choose(parked)
            self.assertIsNotNone(p)
            verify_region_pose(self.c,p,parked,self.grid,self.reserve)
            parked.append(rack_pose(p,self.c['load_center_m']))
        self.assertEqual(len(set(parked)),4)

    def test_new_hit_invalidates_chosen_goal_and_unknown_region_blocks(self):
        first=self.choose([])
        self.grid.data[grid_cell(self.grid,first)]=100
        self.assertNotEqual(self.choose([]),first)
        with self.assertRaises(RuntimeError):
            verify_region_pose(self.c,first,[],obstacle_grid(self.grid),self.reserve)
        self.grid.data[:]=[-1]*15100
        self.assertIsNone(self.choose([]))
