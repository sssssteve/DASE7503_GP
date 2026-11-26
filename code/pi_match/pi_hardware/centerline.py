"""Optional transfer waypoints along a measured horizontal aisle."""
from dataclasses import dataclass
import math

from course_clearance import MARGIN, NAV_GOAL_TOLERANCE_M, box_distance, check_corridor, corners, relative, verify_travel


@dataclass(frozen=True)
class Centerline:
    y: float
    entry_x: float
    end_x: float

    def rack_gate(self, approach):
        left, right = sorted((self.entry_x, self.end_x))
        x, _, yaw = approach
        return min(right, max(left, x)), self.y, yaw


def start_passage(config):
    """Cross between P2 and P4 before turning toward P1, with the nose facing the finish wall."""
    by_id={r['code'][:5]:corners(r['rack_hint'],config['rack_size_m']) for r in config['racks']}
    lower,upper=by_id['RACKB'],by_id['RACKD']
    # Offset the transit line by the existing clearance margin away from P4.
    y=(max(p[1] for p in lower)+min(p[1] for p in upper))/2-MARGIN
    clearance=config['radius_empty_m']+NAV_GOAL_TOLERANCE_M
    return ((max(p[0] for p in lower+upper)+clearance,y,math.pi),
            (sum(p[0] for p in upper)/len(upper),y,math.pi),
            (min(p[0] for p in lower+upper)-clearance,y,math.pi))


def check_detour_scan(points,origin,direction,turn,travel,body_size,*,pure_strafe=False,center_x=0.):
    """Check the turning corners and require observed free space across the swept strip."""
    steps=max(1,math.ceil(abs(turn)/math.radians(3)))
    corner_pad=math.hypot(*body_size)/2*abs(turn)/(2*steps)+.005
    for i in range(steps+1):
        yaw=turn*i/steps
        pose=(center_x*math.cos(yaw),center_x*math.sin(yaw),yaw)
        if any(box_distance(p,pose,body_size)<=corner_pad for p in points):
            raise ValueError('转动时车角或当前车身靠近回波')
    frame=(center_x if pure_strafe else 0.,0.,direction)
    # Bound translation even if the local controller turns and translates together.
    diameter=2*math.hypot(body_size[0]/2+abs(center_x),body_size[1]/2)
    sweep_size=(body_size[1],body_size[0]) if pure_strafe else (diameter,diameter)
    check_corridor([relative(p,frame) for p in points],relative(origin,frame),travel,sweep_size)


def verify_aisle_leg(start, goal, radius, grid, posts, racks, rack_size):
    """Check the full segment with the footprint suitable for this trip."""
    x, y = start[:2]
    delta = goal[0]-x, goal[1]-y, 0.
    verify_travel((x, y, 0.), delta, radius, grid, posts, racks, rack_size, horizon=1.)
