"""Read-only field overlay when navigation isn't supplying map TF."""
from collections import Counter
import math
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'pi_hardware'))
from official_localization.geometry import WALLS, wall_match
from field_observations import wall_segments

WIDTH,HEIGHT=3.02,2.


def offsets(values,size,low,high):
    # Each projected return votes for the robot coordinate from either opposing wall.
    votes=Counter(round((edge-v)/.04) for v in values for edge in (0,size)
                  if low<=edge-v<=high)
    return [.04*k for k,_ in votes.most_common(3)]


def residual(points,pose):
    x,y,a=pose
    c,s=math.cos(a),math.sin(a)
    error=0.
    for u,v in points:
        px,py=x+c*u-s*v,y+s*u+c*v
        if -.05<=px<=WIDTH+.05 and -.05<=py<=HEIGHT+.05:
            error+=min(.15,abs(px),abs(WIDTH-px),abs(py),abs(HEIGHT-py))
        else:
            error+=.15
    return error/len(points)


def start_side_seed(points):
    candidates=[]
    for deg in range(-180,180,5):
        a=math.radians(deg)
        c,s=math.cos(a),math.sin(a)
        xs=offsets([c*u-s*v for u,v in points],WIDTH,WIDTH/2,WIDTH-.095)
        ys=offsets([s*u+c*v for u,v in points],HEIGHT,.095,HEIGHT-.095)
        for x in xs:
            for y in ys:
                pose=(x,y,a)
                candidates.append((residual(points,pose),pose))
    if not candidates:return None
    score,best=min(candidates)
    if score>.065:return None
    for rival,pose in candidates:
        if rival>=score+.012:continue
        if math.dist(pose[:2],best[:2])>=.12 or abs(math.remainder(pose[2]-best[2],math.tau))>=math.radians(15):
            return None
    return best


def wall_support(points):
    # Ignore compact leg clusters when evaluating a walls-only reference map.
    keep=set()
    for i in range(len(points)-4):
        j=i+4
        while j+1<len(points) and math.dist(points[i],points[j])<.22:j+=1
        segment=points[i:j+1]
        x,y=segment[0]
        dx,dy=segment[-1][0]-x,segment[-1][1]-y
        length=math.hypot(dx,dy)
        if length<.22 or any(math.dist(a,b)>.18 for a,b in zip(segment,segment[1:])):continue
        if all(abs(dx*(v-y)-dy*(u-x))<=.025*length for u,v in segment):
            keep.update(range(i,j+1))
    return [points[i] for i in sorted(keep)]


def refine_supported_pose(support,points,prior):
    # A rack face close to a wall can pull a full-scan fit away from long wall segments.
    fit=wall_match(points,prior)
    seeds=[prior,fit[0]] if fit else [prior]
    pose=min(seeds,key=lambda q:residual(support,q))
    for step,angle in ((.025,2.),(.0125,1.),(.006,.5)):
        x,y,a=pose
        candidates=[(x+dx,y+dy,math.remainder(a+math.radians(da),math.tau))
                    for dx in (0.,-step,step) for dy in (0.,-step,step)
                    for da in (0.,-angle,angle) if 0<x+dx<WIDTH and 0<y+dy<HEIGHT]
        if not candidates:return None
        pose=min(candidates,key=lambda q:residual(support,q))
    if residual(support,pose)>.065:return None
    x,y,a=pose
    c,s=math.cos(a),math.sin(a)
    projected=[(x+c*u-s*v,y+s*u+c*v) for u,v in points]
    segments=wall_segments(projected)
    labels={seg['wall'] for seg in segments if abs(seg['offset'])<=.045 and
            math.dist(*seg['ends'])>=.30}
    return pose if len({axis for axis,_,label in WALLS if label in labels})==2 else None


class WallPreview:
    def __init__(self):
        self.prior=None
        self.hits=0
        self.last_stamp=None
        self.last_good_stamp=None
        self.last_result=None

    def estimate(self,sample):
        stamp=sample.get('scan_stamp')
        if not stamp or len(sample.get('filtered',[]))<32:
            self.hits=0
            return None
        if stamp==self.last_stamp:return self.last_result
        if ((self.last_stamp is not None and stamp<self.last_stamp) or
                (self.last_good_stamp is not None and stamp-self.last_good_stamp>=1_500_000_000)):
            self.prior=None
            self.hits=0
        self.last_stamp=stamp
        self.last_result=None
        pts=sample['filtered']
        support=wall_support(pts)
        support=support[::max(1,round(len(support)/110))]
        if len(support)<24:return None
        prior=self.prior or start_side_seed(support)
        pose=refine_supported_pose(support,pts,prior) if prior else None
        if pose is None:return None
        stable=(self.prior is not None and math.dist(pose[:2],self.prior[:2])<.08 and
                abs(math.remainder(pose[2]-self.prior[2],math.tau))<math.radians(10))
        self.hits=self.hits+1 if stable else 1
        self.prior=pose
        self.last_good_stamp=stamp
        self.last_result=pose if self.hits>=3 else None
        return self.last_result
