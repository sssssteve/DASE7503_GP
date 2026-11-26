import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from geometry import filter_ranges, rack_legs, wall_match, WALLS


def scan_at(pose, omit=None):
    x, y, heading = pose
    points = []
    for i in range(-180, 181, 2):
        angle = heading + math.radians(i)
        axis = math.cos(angle), math.sin(angle)
        hits = [((edge-(x,y)[k])/axis[k], label) for k,edge,label in WALLS
                if abs(axis[k]) > 1e-8 and (edge-(x,y)[k])/axis[k] > 0]
        distance, label = min(hits)
        if label != omit:
            a = math.radians(i)
            points.append((distance*math.cos(a), distance*math.sin(a)))
    return points


class GeometryTest(unittest.TestCase):
    def test_filter_preserves_neighbor_returns_and_angles(self):
        ranges = [.10, .12, 1.00, 1.03, math.nan, 2.2, math.inf]
        filtered = filter_ranges(ranges, .01, .03, 12.)
        self.assertEqual(filtered, [.10, .12, 1., 1.03, math.inf, math.inf, math.inf])

    def test_range_adaptive_keeps_a_two_beam_leg_but_rejects_near_depth_jump(self):
        echo=[.20,.27,math.nan,1.,1.025,math.nan,math.nan,2.]
        clean=filter_ranges(echo,.014,.03,12.,near_gap=.03)
        self.assertEqual(clean,[math.inf,math.inf,math.inf,1.,1.025,math.inf,math.inf,math.inf])
        self.assertEqual(filter_ranges(echo,.014,.03,12.)[:2],[.20,.27])

    def test_same_front_wall_after_backwards_movement(self):
        for x in (2.68, 2.33, 1.94):
            truth = x, 1.02, 0.
            points = scan_at(truth) + [(.28, .55), (-.39, .33)]
            candidate, _, front = wall_match(points, (x+.06, .97, math.radians(4)))
            self.assertLess(math.dist(truth[:2], candidate[:2]), .025)
            self.assertLess(abs(candidate[2]), math.radians(3))
            self.assertEqual(front, 'START 墙')

    def test_identifies_destination_wall_when_actual_heading_changes(self):
        truth = 1.20, .83, math.pi
        self.assertEqual(wall_match(scan_at(truth), truth)[2], '终点区墙')

    def test_single_wall_cannot_reset_pose(self):
        points = [(.34, i*.016) for i in range(-40,40)]
        self.assertIsNone(wall_match(points, (2.68, 1.02, 0.)))

    def test_no_invented_legs_when_two_are_occluded(self):
        xs = [(1.06, yc+d) for yc in (1.27,1.65) for d in (-.005,.005)]
        p1 = rack_legs(xs)[0]
        self.assertEqual(len(p1['legs']), 2)
        self.assertFalse(p1['confirmed'])
        xs += [(1.20, yc+d) for yc in (1.65,1.27) for d in (-.005,.005)]
        p1 = rack_legs(xs)[0]
        self.assertEqual(len(p1['legs']), 4)
        self.assertTrue(p1['confirmed'])


if __name__ == '__main__':
    unittest.main()
