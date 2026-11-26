import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from centerline import Centerline, check_detour_scan, verify_aisle_leg


class CenterlineTest(unittest.TestCase):
    def test_rack_gate_projects_to_aisle_and_stays_between_its_ends(self):
        aisle=Centerline(1.,2.10,.85)
        self.assertEqual(aisle.rack_gate((1.76,.63,math.pi)),(1.76,1.,math.pi))
        self.assertEqual(aisle.rack_gate((2.30,1.60,math.pi)),(2.10,1.,math.pi))

    def test_oblique_return_outside_actual_strafe_envelope(self):
        rays=[(math.cos(math.radians(i)),math.sin(math.radians(i))) for i in range(360)]
        sample=rays+[(-.163,.455),(-.171,.454),(-.177,.450),(-.188,.440),(-.190,.430)]
        args=((.10,0.),math.pi/2,0.,.25,(.28,.19))
        with self.assertRaisesRegex(RuntimeError,'路径内'):
            check_detour_scan(sample,*args)
        check_detour_scan(sample,*args,pure_strafe=True)
        with self.assertRaisesRegex(RuntimeError,'路径内'):
            check_detour_scan(sample+[(.10,.24)],*args,pure_strafe=True)
        with self.assertRaisesRegex(RuntimeError,'缺少雷达覆盖'):
            check_detour_scan([],*args,pure_strafe=True)

    def test_carrying_rejects_a_rack_in_the_middle_of_an_apparently_clear_aisle(self):
        grid=NS(info=NS(width=151,height=100,resolution=.02,
                        origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),
                data=[0]*15100)
        start=(2.10,1.,math.pi)
        goal=(1.50,1.,math.pi)
        other_racks=[(1.81,.63,math.pi)]
        size=(.14,.38)
        verify_aisle_leg(start,goal,.12,grid,[],other_racks,size)
        with self.assertRaisesRegex(RuntimeError,'已登记的货架'):
            verify_aisle_leg(start,goal,math.hypot(.07,.19),grid,[],other_racks,size)


if __name__=='__main__':
    unittest.main()
