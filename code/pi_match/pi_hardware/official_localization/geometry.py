"""Geometry for the official 3.02 × 2.00 m reference field (metres)."""
from itertools import combinations
import math
from statistics import median

WALLS = ((0, 0., '终点区墙'), (0, 3.02, 'START 墙'),
         (1, 0., '下侧墙'), (1, 2., '上侧墙'))
RACKS = (('P1', 1.06, 1.27), ('P2', 1.85, 1.41),
         ('P3', 1.25, .13), ('P4', 1.74, .44))


def filter_ranges(ranges, step, minimum, maximum, near_gap=None, *, body_edge_bins=()):
    valid = [math.isfinite(r) and minimum <= r < maximum for r in ranges]
    full_circle = abs(step)*len(ranges) >= math.tau - 1.5*abs(step)
    result = [math.inf]*len(ranges)
    def neighbors(i):
        r=ranges[i]
        for offset in (-2, -1, 1, 2):
            j = i + offset
            if full_circle:
                j %= len(ranges)
            if 0 <= j < len(ranges) and j != i and valid[j]:
                s = ranges[j]
                chord = 2*abs(math.sin(offset*step/2))
                distance = (r-s)**2 + r*s*chord**2
                limit = .10 if near_gap is None else min(.10,near_gap+2*min(r,s)*chord)
                if distance <= limit**2:
                    yield j,distance
    for i, r in enumerate(ranges):
        if not valid[i]:continue
        links=neighbors(i)
        first=next(links,None)
        if first is None:continue
        j,distance=first
        if (i in body_edge_bins and j in body_edge_bins and distance<=.01**2 and next(links,None) is None
                and all(k==i for k,_ in neighbors(j))):
            continue
        result[i]=r
    return result


def wall_match(points, prior):
    """Match two perpendicular walls in the vicinity of the predicted pose."""
    x, y, angle = prior
    best = None
    for degrees in range(-8, 9, 2):
        a = angle + math.radians(degrees)
        c, s = math.cos(a), math.sin(a)
        projected = [(x+c*u-s*v, y+s*u+c*v) for u, v in points]
        fits = [[], []]
        for axis, edge, label in WALLS:
            limit = (2., 3.02)[axis]
            hits = [(p[axis]-edge, p[1-axis]) for p in projected
                    if abs(p[axis]-edge) <= .15 and .04 <= p[1-axis] <= limit-.04]
            for residual in sorted({round(r/.006)*.006 for r, _ in hits}):
                group = [(r, t) for r, t in hits if abs(r-residual) <= .024]
                if len(group) < 8:
                    continue
                lateral = [t for _, t in group]
                sectors = {int(t/.08) for t in lateral}
                if len(sectors) < 4 or max(lateral)-min(lateral) < .32:
                    continue
                shift = median(r for r, _ in group)
                error = median(abs(r-shift) for r, _ in group)
                fits[axis].append((len(sectors), error, shift, label))
                if len(group) == len(hits):
                    break
        if not all(fits):
            continue
        a_fit, b_fit = [max(group, key=lambda f: (f[0], -f[1], -abs(f[2]))) for group in fits]
        candidate = x-a_fit[2], y-b_fit[2], math.remainder(a, math.tau)
        if math.dist(candidate[:2], prior[:2]) > .14 or not (0 < candidate[0] < 3.02 and 0 < candidate[1] < 2):
            continue
        # Use all supported perpendicular hits, with a small tie-break toward the prior.
        score = (a_fit[0]+b_fit[0]) - 180*(a_fit[1]+b_fit[1]) - .08*abs(degrees)
        if best is None or score > best[0]:
            best = score, candidate, (a_fit[3], b_fit[3])
    if best is None:
        return None
    _, pose, labels = best
    x, y, a = pose
    c, s = math.cos(a), math.sin(a)
    _, axis, edge, forward = min(((edge-(x, y)[axis])/(c, s)[axis], axis, edge, label)
                                 for axis, edge, label in WALLS
                                 if abs((c, s)[axis]) > 1e-6 and (edge-(x, y)[axis])/(c, s)[axis] > 0)
    along = []
    for u, v in points:
        p = x+c*u-s*v, y+s*u+c*v
        if abs(p[axis]-edge) <= .035 and .04 <= p[1-axis] <= (2., 3.02)[axis]-.04:
            along.append(p[1-axis])
    seen = len(along) >= 8 and len({int(t/.08) for t in along}) >= 4 and max(along)-min(along) >= .32
    return pose, labels, forward if seen else None


def leg_centers(points):
    """Return centres of compact groups of ordered scan returns."""
    clusters = []
    for p in points:
        if not clusters or math.dist(p, clusters[-1][-1]) > .055:
            clusters.append([])
        clusters[-1].append(p)
    if len(clusters) > 1 and math.dist(clusters[0][0], clusters[-1][-1]) <= .055:
        clusters[0] = clusters.pop() + clusters[0]
    centers = []
    for group in clusters:
        if len(group) < 2:
            continue
        xs, ys = zip(*group)
        if math.hypot(max(xs)-min(xs), max(ys)-min(ys)) <= .085:
            centers.append((median(xs), median(ys)))
    return centers


def rack_legs(points, layout=None):
    """Report only observed leg candidates near the diagram's rack zones."""
    centers = leg_centers(points)
    racks = []
    if layout is None:
        layout=[dict(name=name,center=[x+.07,y+.19],size=[.14,.38],yaw=0.) for name,x,y in RACKS]
    for r in layout:
        name=r['name']
        cx,cy=r['center']
        w,h=r['size']
        a=r['yaw']
        corners=[(cx+u*math.cos(a)-v*math.sin(a),cy+u*math.sin(a)+v*math.cos(a))
                 for u in (-w/2,w/2) for v in (-h/2,h/2)]
        legs = []
        # These offsets are estimated from the nominal 0.14 × 0.38 m footprint.
        for corner in corners:
            near = [p for p in centers if math.dist(p, corner) <= .06]
            if len(near) == 1:
                legs.append(near[0])
        confirmed = False
        if len(legs) == 4:
            sides = sorted(math.dist(a,b) for a,b in combinations(legs, 2))
            expected = sorted([w,w,h,h,math.hypot(w,h),math.hypot(w,h)])
            confirmed = all(abs(a-b) <= .055 for a,b in zip(sides, expected))
        racks.append(dict(name=name, legs=legs, confirmed=confirmed))
    return racks
