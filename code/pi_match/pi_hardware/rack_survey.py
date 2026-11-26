"""Estimate a rack pose from four distinct LiDAR leg centres."""
import math
from statistics import mean

from course_clearance import corners, world
from official_localization.geometry import leg_centers


def stable_legs(scans):
    """Keep each compact return only when supported by another scan."""
    tracks = []
    for frame, points in enumerate(scans):
        for point in leg_centers(points):
            nearby = [t for t in tracks if frame not in t and math.dist(next(iter(t.values())), point) <= .026]
            if nearby:
                min(nearby, key=lambda t: math.dist(next(iter(t.values())), point))[frame] = point
            else:
                tracks.append({frame: point})
    return [tuple(map(mean, zip(*t.values()))) for t in tracks if len(t) >= 2]


def fit_rack(legs, hint, size):
    """Match a rectangle near the searched rack, returning its pose and measured corners."""
    offsets = corners((0.,0.,0.), size)
    span = math.hypot(*size)/2 + .20
    legs = [p for p in legs if math.dist(p, hint[:2]) <= span]
    if len(legs) < 4:
        return None
    candidates = {}
    for degrees in range(-16, 17, 4):
        heading = hint[2] + math.radians(degrees)
        rotated = [world(p, (0.,0.,heading)) for p in offsets]
        for px,py in legs:
            for dx,dy in rotated:
                centre = px-dx,py-dy
                if math.dist(centre, hint[:2]) > .23:
                    continue
                prediction = [world(p, (*centre,heading)) for p in offsets]
                nearest = tuple(min(range(len(legs)), key=lambda i: math.dist(legs[i],p)) for p in prediction)
                if nearest in candidates or len(set(nearest)) != 4:
                    continue
                observed = [legs[i] for i in nearest]
                cx,cy = map(mean, zip(*observed))
                if math.dist((cx,cy), hint[:2]) > .20:
                    continue
                dot = sum(x*(u-cx)+y*(v-cy) for (x,y),(u,v) in zip(offsets,observed))
                cross = sum(x*(v-cy)-y*(u-cx) for (x,y),(u,v) in zip(offsets,observed))
                angle = math.atan2(cross,dot)
                if abs(math.remainder(angle-hint[2],math.tau)) > math.radians(16):
                    continue
                pose = cx,cy,angle
                errors = [math.dist(world(p,pose),q) for p,q in zip(offsets,observed)]
                if max(errors) <= .026:
                    candidates[nearest] = mean(errors),pose,observed
    if not candidates:
        return None
    score, pose, measured = min(candidates.values(), key=lambda c:c[0])
    for other_score, other_pose, _ in candidates.values():
        if other_score-score <= .008 and (
                math.dist(other_pose[:2],pose[:2]) > .05 or
                abs(math.remainder(other_pose[2]-pose[2],math.tau)) > math.radians(10)):
            return None
    return pose, measured


def insertion_error(pose, offset):
    """Rack pose in base_link; return forward travel, lateral error and yaw."""
    x,y,a=pose
    dx,dy=world(offset,(0.,0.,a))
    return x-dx,y-dy,math.remainder(a,math.tau)


def measure_spacing(legs, hint, outer, vehicle_width):
    """Find a unique observed rectangle compatible with the rack envelope."""
    from itertools import combinations
    near=[p for p in legs if math.dist(p,hint[:2])<=.20+math.hypot(*outer)/2]
    if len(near)>16:
        return None
    matches=[]
    for four in combinations(near,4):
        for a in four:
            for b in four:
                dx,dy=b[0]-a[0],b[1]-a[1]
                short=math.hypot(dx,dy)
                if not .04<short<=outer[0]+.01:
                    continue
                yaw=math.atan2(dy,dx)
                if abs(math.remainder(yaw-hint[2],math.tau))>math.radians(16):
                    continue
                rest=[p for p in four if p!=a and p!=b]
                if len(rest)!=2:continue
                c,d=sorted(rest,key=lambda p:math.dist(p,a))
                vx,vy=c[0]-a[0],c[1]-a[1]
                long=math.hypot(vx,vy)
                if not vehicle_width+.02<long<=outer[1]+.01:continue
                if abs(vx*dx+vy*dy)/short>.02 or math.dist((b[0]+vx,b[1]+vy),d)>.025:
                    continue
                cx,cy=map(mean,zip(*four))
                if math.dist((cx,cy),hint[:2])>.20:continue
                matches.append(((short+math.dist(c,d))/2,(long+math.dist(b,d))/2))
                break
            else:
                continue
            break
        if len(matches)>1:
            return None
    return matches[0] if matches else None


def observed_opening(points, origin, pose, depth):
    """Width around the centre traversed by contiguous clear rays at both rack faces."""
    from course_clearance import relative
    ox,oy=relative(origin,pose)
    front,back=-depth/2,depth/2
    if ox>=front:return 0.
    crossings=[[],[]]
    for point in points:
        x,y=relative(point,pose)
        if x<=back+.02:continue
        for row,plane in zip(crossings,(front,back)):
            row.append(oy+(y-oy)*(plane-ox)/(x-ox))
    widths=[]
    for row in crossings:
        band=[]
        best=0.
        for value in sorted(row):
            if band and value-band[-1]>.025:
                band=[]
            band.append(value)
            if band[0]<0<value:
                best=max(best,2*min(-band[0],value))
        widths.append(best)
    return min(widths)
