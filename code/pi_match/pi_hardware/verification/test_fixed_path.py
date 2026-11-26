import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from course_clearance import grid_cell, verify_path


class FixedPathTest(unittest.TestCase):
    def test_obstacle_between_waypoints_clips_loaded_outline(self):
        grid=NS(info=NS(width=151,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15100)
        route=[(2.1,1.115),(1.5,1.115)]
        for value in (-1,100):
            grid.data[grid_cell(grid,(1.80,.89))]=value
            verify_path(route,.158,grid,[],[],(.14,.38))
            with self.assertRaises(RuntimeError):
                verify_path(route,.203,grid,[],[],(.14,.38))

    def test_nominal_loaded_passage(self):
        grid=NS(info=NS(width=151,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15100)
        p4=(1.81,.63,math.pi)
        verify_path([(2.10,1.115),(1.02,1.115)],.203,grid,[],[p4],(.14,.38))
        with self.assertRaises(RuntimeError):
            verify_path([(2.10,1.),(1.02,1.)],.203,grid,[],[p4],(.14,.38))
