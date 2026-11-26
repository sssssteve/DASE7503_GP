"""Measured overlays in map coordinates. These observations do not command the vehicle."""
from collections import deque
from itertools import combinations
from statistics import median
import math

from official_localization.geometry import WALLS
from rack_survey import stable_legs
from course_clearance import corners


def wall_segments(points):
    segments=[]
    for axis,edge,label in WALLS:
        limit=(2.,3.02)[axis]
        hits=sorted((p[1-axis],p[axis]-edge) for p in points
                    if abs(p[axis]-edge)<.11 and 0<=p[1-axis]<=limit)
        runs=[]
        for along,offset in hits:
            if not runs or along-runs[-1][-1][0]>.14 or abs(offset-runs[-1][-1][1])>.045:
                runs.append([])
            runs[-1].append((along,offset))
        for run in runs:
            length=run[-1][0]-run[0][0]
            if len(run)<5 or length<.20:continue
            # Fit the measured line using separated ends; keep only its observed span.
            ends=max(1,len(run)//4)
            left,right=run[:ends],run[-ends:]
            dx=median(t for t,_ in right)-median(t for t,_ in left)
            if dx<=.10:continue
            slope=(median(v for _,v in right)-median(v for _,v in left))/dx
            if abs(slope)>.14:continue
            offset=median(v-slope*t for t,v in run)
            inliers=[t for t,v in run if abs(v-offset-slope*t)<=.025]
            if len(inliers)<5 or inliers[-1]-inliers[0]<.20:continue
            ends=[]
            for t in (inliers[0],inliers[-1]):
                normal=edge+offset+slope*t
                ends.append([normal,t] if axis==0 else [t,normal])
            segments.append(dict(wall=label,ends=ends,offset=median(v for _,v in run)))
    return segments


def rack_rectangles(legs):
    rectangles=[]
    for i,p in enumerate(legs):
        near=[q for q in legs[i+1:] if math.dist(p,q)<.45]
        if len(near)>15:continue
        for triple in combinations(near,3):
            a,b,d=sorted(triple,key=lambda q: math.dist(p,q))
            short,long=math.dist(p,a),math.dist(p,b)
            if not (.07<=short<=.16 and .24<=long<=.40):continue
            if abs((a[0]-p[0])*(b[0]-p[0])+(a[1]-p[1])*(b[1]-p[1]))>short*.025:continue
            if math.hypot(a[0]+b[0]-p[0]-d[0],a[1]+b[1]-p[1]-d[1])>.026:continue
            group=[p,a,d,b]
            center=[sum(q[k] for q in group)/4 for k in (0,1)]
            rectangles.append(dict(legs=group,center=center))
    # Ambiguous rectangles sharing a leg stay unconfirmed.
    return [r for r in rectangles if not any(other is not r and
            any(q in other['legs'] for q in r['legs']) for other in rectangles)]


def reference_support(legs,layout):
    targets=[(zone['name'],i,p) for zone in layout for i,p in
             enumerate(corners((*zone['center'],zone['yaw']),zone['size']))]
    matches={zone['name']:set() for zone in layout}
    for leg in legs:
        near=sorted((math.dist(leg,p),name,i) for name,i,p in targets)
        if not near or near[0][0]>.08:continue
        if len(near)>1 and near[1][0]-near[0][0]<.015:continue
        _,name,i=near[0]
        matches[name].add(i)
    return {name:len(indices) for name,indices in matches.items()}


class FieldObservations:
    def __init__(self):
        self.frames=deque(maxlen=3)
        self.stamp=0
        self.source=None

    def update(self,points,stamp,source,layout=()):
        if not 0<stamp-self.stamp<800_000_000 or self.source!=source:self.frames.clear()
        self.stamp,self.source=stamp,source
        self.frames.append(points)
        legs=stable_legs(self.frames) if len(self.frames)>=2 else []
        # A short segment on a field boundary is not evidence of a rack leg.
        interior=[p for p in legs if .06<p[0]<2.96 and .06<p[1]<1.94]
        rectangles=rack_rectangles(interior)
        for rack in rectangles:
            nearby=[zone for zone in layout if math.dist(zone['center'],rack['center'])<.15]
            if len(nearby)==1:
                zone=nearby[0]
                rack['near_zone']=zone['name']
                rack['offset_m']=math.dist(zone['center'],rack['center'])
        return dict(wall_segments=wall_segments(points),leg_candidates=interior,
                    measured_racks=rectangles,reference_hits=reference_support(interior,layout))
