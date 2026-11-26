import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from course_clearance import corners
from rack_survey import fit_rack, stable_legs


def returns(legs, drift=0.):
    return [(x+dx+drift,y) for x,y in legs for dx in (-.006,0.,.006)]


class RackSurveyTest(unittest.TestCase):
    def setUp(self):
        self.size=(.105,.34)
        self.hint=(1.12,1.46,math.pi)
        self.actual=(1.17,1.42,math.pi-.13)
        self.legs=corners(self.actual,self.size)

    def test_two_views_supply_distinct_stable_legs_and_correct_pose(self):
        front,rear=self.legs[:2],self.legs[2:]
        scans=[returns(front),returns(front,.005),returns(rear),returns(rear,-.005)]
        fit, measured=fit_rack(stable_legs(scans),self.hint,self.size)
        self.assertEqual(len(measured),4)
        self.assertLess(math.dist(fit[:2],self.actual[:2]),.005)
        self.assertLess(abs(math.remainder(fit[2]-self.actual[2],math.tau)),.04)

    def test_transient_fourth_leg_and_three_real_legs_do_not_pass(self):
        scans=[returns(self.legs[:3]),returns(self.legs[:3],.003),returns(self.legs)]
        self.assertIsNone(fit_rack(stable_legs(scans),self.hint,self.size))

    def test_equally_plausible_rack_positions_are_rejected(self):
        shifted=[(x+.17,y) for x,y in self.legs]
        hint=(1.24,1.46,math.pi)
        self.assertIsNone(fit_rack(self.legs+shifted,hint,self.size))


if __name__=='__main__':
    unittest.main()
