import math
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from course_clearance import corners
from rack_survey import measure_spacing, observed_opening


class AutoSurveyTest(unittest.TestCase):
    def test_four_observations_measure_spacing_three_do_not(self):
        rack=(1.1,1.4,math.pi-.09)
        legs=corners(rack,(.11,.34))
        measured=measure_spacing(legs,(1.1,1.4,math.pi),(.14,.38),.19)
        for actual,expected in zip(measured,(.11,.34)):
            self.assertAlmostEqual(actual,expected)
        self.assertIsNone(measure_spacing(legs[:3],rack,(.14,.38),.19))

    def test_opening_requires_rays_through_both_faces(self):
        rays=[(.8,y/100.) for y in range(-45,46)]
        pose=(.34,0.,0.)
        self.assertGreater(observed_opening(rays,(0.,0.),pose,.14),.23)
        self.assertEqual(observed_opening([(.30,y) for _,y in rays],(0.,0.),pose,.14),0.)
        self.assertEqual(observed_opening([r for r in rays if r[1]>.1],(0.,0.),pose,.14),0.)
