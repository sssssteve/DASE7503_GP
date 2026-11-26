"""Enumerate safe unloading poses in the official west-side destination area."""
from array import array
from copy import copy
import math

from course_clearance import (
    MARGIN, TRAVEL_MARGIN, centered_body_radius, box_distance, rack_pose, verify_drop, verify_retreat, verify_travel,
)


def obstacle_grid(costmap):
    # Nav2 has already inflated intermediate costs. Sweep our own measured footprint
    # against lethal and unknown cells once, without inflating them a second time.
    grid=copy(costmap)
    grid.data=array('b',(100 if value<0 or value==100 else 0 for value in costmap.data))
    return grid


def verify_region_pose(c, base, occupied, grid, reserved):
    placed=rack_pose(base,c['load_center_m'])
    size=c['rack_size_m']
    verify_drop(c,placed,occupied)
    verify_retreat(base,-c['retreat_m'],c,occupied)
    # Keep the final END viewpoint and remaining fixed drop goals usable.
    for goal,radius in reserved:
        if box_distance(goal,placed,size)<=radius+TRAVEL_MARGIN:
            raise ValueError('落架会占用后续目标')
    verify_travel(base,(0.,0.,0.),c['radius_loaded_m'],grid,c['qr_posts'],occupied,size)
    center=rack_pose(base,(c.get('body_center_x_m',0.),0.))
    verify_travel(center,(-c['retreat_m'],0.,0.),centered_body_radius(c),grid,c['qr_posts'],occupied,size,horizon=1.)


def candidates(c, occupied, grid, reserved, tick):
    x0,y0,x1,y1=c['destination_bounds']
    # In the official map the entrance is on the right. Face west so reversing
    # clears the rack toward the entrance. Search rear columns and outer rows first.
    step=max(.05,grid.info.resolution)
    ys=[y0+(i+.5)*step for i in range(math.ceil((y1-y0)/step))]
    ys.sort(key=lambda y:-abs(y-(y0+y1)/2))
    for i in range(math.ceil((x1-x0)/step)):
        tick()
        for y in ys:
            centre=x0+(i+.5)*step,y
            dx,dy=c['load_center_m']
            base=(centre[0]+dx,centre[1]+dy,math.pi)
            try:
                verify_region_pose(c,base,occupied,grid,reserved)
            except (RuntimeError,ValueError):
                continue
            yield base
