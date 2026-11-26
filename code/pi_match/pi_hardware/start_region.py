"""Estimate a stationary base pose on a fixed-structure occupancy grid."""
import math

import numpy as np


def scan_xy(scan, mount):
    x, y, _, roll, pitch, yaw = mount
    r = np.asarray(scan.ranges)
    bins = np.flatnonzero(np.isfinite(r) & (r >= scan.range_min) & (r < scan.range_max))
    angles = scan.angle_min + scan.angle_increment * bins
    u, v = r[bins] * np.cos(angles), r[bins] * np.sin(angles)
    px = math.cos(pitch) * u + math.sin(pitch) * math.sin(roll) * v
    py = math.cos(roll) * v
    u=x+math.cos(yaw)*px-math.sin(yaw)*py
    v=y+math.sin(yaw)*px+math.cos(yaw)*py
    return bins,u,v


def inside_body(u, v, body_size, padding=0., center_x=0.):
    u=u-center_x
    if padding:
        dx=np.maximum(np.abs(u)-body_size[0]/2,0.)
        dy=np.maximum(np.abs(v)-body_size[1]/2,0.)
        # Pad the sides and rear for chassis echoes, but keep returns ahead of the nose.
        return (u<=body_size[0]/2+1e-9) & (np.hypot(dx,dy)<=padding+1e-9)
    return (np.abs(u)<=body_size[0]/2)&(np.abs(v)<=body_size[1]/2)


def scan_mask(u, v, body_size, box=None, padding=0., center_x=0.):
    inside=inside_body(u,v,body_size,padding,center_x)
    if box is None:
        return inside
    xmin,ymin,xmax,ymax=box
    return inside | ((xmin<=u)&(u<=xmax)&(ymin<=v)&(v<=ymax))


def mask_body(scan, mount, body_size, box=None):
    bins,u,v=scan_xy(scan,mount)
    inside=scan_mask(u,v,body_size,box)
    for index in bins[inside]:
        scan.ranges[int(index)]=math.nan


def scan_components(scan,mount,body_size,box=None,near_gap=None,echo_windows=(),body_padding=0.,center_x=0.):
    from official_localization.geometry import filter_ranges
    bins,x,y=scan_xy(scan,mount)
    masked=scan_mask(x,y,body_size,box,body_padding,center_x)
    ranges=np.asarray(scan.ranges,dtype=float).copy()
    if echo_windows:
        deg=(np.degrees(scan.angle_min+scan.angle_increment*bins)+180.)%360.-180.
        distances=ranges[bins]
        for low,high,rmin,rmax in echo_windows:
            masked |= (low<=deg)&(deg<=high)&(rmin<=distances)&(distances<=rmax)
    ranges[bins[masked]]=math.nan
    # Beside or behind the hull a pair of almost coincident beams can support only each other.
    # Leave frontal and distant two-beam leg observations available for docking.
    half_length,half_width=body_size[0]/2,body_size[1]/2
    body_x=x-center_x
    beside=(np.abs(body_x)<=half_length)&(np.abs(y)>half_width)&(np.abs(y)<=half_width+.05)
    behind=(body_x < -half_length)&(body_x >= -half_length-.05)&(np.abs(y)<=half_width)
    supported=filter_ranges(ranges,scan.angle_increment,scan.range_min,scan.range_max,near_gap,
                            body_edge_bins=set(bins[beside|behind]))
    isolated=~masked & ~np.isfinite(np.asarray(supported)[bins])
    return bins,x,y,masked,isolated


def filter_scan(scan,mount,body_size,box=None,near_gap=None,echo_windows=(),body_padding=0.,center_x=0.):
    bins,_,_,masked,isolated=scan_components(scan,mount,body_size,box,near_gap,echo_windows,body_padding,center_x)
    for i in bins[masked|isolated]:
        scan.ranges[int(i)]=math.nan


def scan_points(scan, mount, body_size=None):
    _,u,v=scan_xy(scan,mount)
    if len(u)<32:
        raise ValueError('雷达有效回波不足')
    if body_size is not None:
        outside=~inside_body(u,v,body_size)
        u,v=u[outside],v[outside]
        if len(u)<32:raise ValueError('剔除车身范围后回波不足')
    keep=np.linspace(0,len(u)-1,min(len(u),120)).astype(int)
    return u[keep],v[keep]


def wall_view(points, odom, current):
    """Put supported straight segments from one stopped scan in the current base frame."""
    xy = np.column_stack(points)
    if len(xy) < 5:
        return np.empty((0, 2))
    wall = np.zeros(len(xy), dtype=bool)
    for i in range(len(xy)-4):
        end=i+4
        while end+1<len(xy) and math.dist(xy[i],xy[end])<.14:
            end+=1
        section=xy[i:end+1]
        dx, dy = section[-1]-section[0]
        span = math.hypot(dx, dy)
        if span < .14 or np.max(np.linalg.norm(np.diff(section, axis=0), axis=1)) > .18:
            continue
        offset = section-section[0]
        if np.max(np.abs(dx*offset[:,1]-dy*offset[:,0])) <= .025*span:
            wall[i:end+1] = True
    angle = odom[2]-current[2]
    c, s = math.cos(angle), math.sin(angle)
    dx, dy = odom[0]-current[0], odom[1]-current[1]
    cr, sr = math.cos(current[2]), math.sin(current[2])
    return xy[wall] @ np.array([[c,s],[-s,c]]) + [cr*dx+sr*dy, -sr*dx+cr*dy]


def combined_walls(views, current):
    walls = np.concatenate([wall_view(points, pose, current) for points, pose in views])
    # Give each patch of wall one vote even when it appears at several stops.
    _, indices = np.unique(np.rint(walls/.035).astype(int), axis=0, return_index=True)
    if len(indices) < 24:
        raise ValueError('可重复观测的墙段不足')
    return walls[indices].T


class StartRegion:
    def __init__(self, grid, bounds):
        origin = grid.info.origin
        q = origin.orientation
        angle = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        self.x, self.y = origin.position.x, origin.position.y
        self.c, self.s = math.cos(angle), math.sin(angle)
        self.res = grid.info.resolution
        w, h = grid.info.width, grid.info.height
        if not (0 < self.res and 0 < w*h <= 300_000):
            raise ValueError('起始区定位需要有效的小场地栅格地图')
        occupancy = np.asarray(grid.data).reshape(h,w)
        occupied = occupancy >= 65
        if not np.any(occupied):
            raise ValueError('地图无固定障碍')
        self.cost = np.full((h,w), .15, dtype=np.float32)
        # Lookup table for distance to a nearby occupied cell; pad without wrapping.
        n = min(math.ceil(.15/self.res), 24)
        padded = np.pad(occupied, n)
        for j in range(-n,n+1):
            for i in range(-n,n+1):
                d = math.hypot(i,j)*self.res
                if d < .15:
                    mask = padded[n+j:n+j+h,n+i:n+i+w]
                    self.cost[mask] = np.minimum(self.cost[mask],d)
        x0,y0,x1,y1 = bounds
        xs,ys = np.meshgrid(np.linspace(x0,x1,math.ceil((x1-x0)/.04)+1),
                            np.linspace(y0,y1,math.ceil((y1-y0)/.04)+1))
        i,j = self.cells(xs.ravel(),ys.ravel())
        inside = (i>=0)&(i<w)&(j>=0)&(j<h)
        valid = inside & (occupancy[j.clip(0,h-1),i.clip(0,w-1)]==0)
        self.positions = np.column_stack([xs.ravel()[valid],ys.ravel()[valid]])
        if not len(self.positions):
            raise ValueError('起始区没有地图已知空闲格')

    def cells(self,x,y):
        x,y=x-self.x,y-self.y
        return (np.floor((self.c*x+self.s*y)/self.res).astype(int),
                np.floor((-self.s*x+self.c*y)/self.res).astype(int))

    def estimate(self, points, headings=None):
        u,v=points
        x,y=self.positions.T
        yaw=np.arange(-math.pi,math.pi,math.radians(5)) if headings is None else np.asarray(headings)
        score=[]
        h,w=self.cost.shape
        for a in yaw:
            i,j=self.cells(x[:,None]+math.cos(a)*u-math.sin(a)*v,
                           y[:,None]+math.sin(a)*u+math.cos(a)*v)
            valid=(i>=0)&(i<w)&(j>=0)&(j<h)
            d=np.where(valid,self.cost[j.clip(0,h-1),i.clip(0,w-1)],.15)
            # Bad returns receive capped penalties. A sizable majority must match.
            score.append(np.mean(d,axis=1))
        score=np.asarray(score)
        k,p=np.unravel_index(np.argmin(score),score.shape)
        best=float(score[k,p])
        separate=(np.hypot(x-x[p],y-y[p])[None,:]>=.12) | (
            np.abs(np.arctan2(np.sin(yaw-yaw[k]),np.cos(yaw-yaw[k])))[:,None]>=math.radians(15))
        rivals=score[separate]
        if best>.065:
            raise ValueError(f'围墙匹配不足（平均截断残差 {best*100:.1f} cm，上限 6.5 cm）')
        if rivals.size and float(rivals.min())-best<.012:
            gap=float(rivals.min())-best
            raise ValueError(f'起始位姿多解（候选分差 {gap*100:.2f} cm，要求至少 1.20 cm）')
        return float(x[p]),float(y[p]),float(yaw[k])
