"""Measured arena geometry shared by match configuration and its motion checks."""
import math

MARGIN = .04
TRAVEL_MARGIN = MARGIN + .01
NAV_GOAL_TOLERANCE_M = .06


def relative(point, pose):
    x,y = point[0]-pose[0],point[1]-pose[1]
    c,s = math.cos(pose[2]),math.sin(pose[2])
    return c*x+s*y,-s*x+c*y


def world(point, pose):
    x,y = point
    c,s = math.cos(pose[2]),math.sin(pose[2])
    return pose[0]+c*x-s*y,pose[1]+s*x+c*y


def corners(pose, size):
    a,b = size[0]/2,size[1]/2
    return [world((i*a,j*b),pose) for i in (-1,1) for j in (-1,1)]


def rack_pose(base, offset):
    return (*world(offset,base),base[2])


def box_distance(point, pose, size):
    x,y = relative(point,pose)
    return math.hypot(max(abs(x)-size[0]/2,0.),max(abs(y)-size[1]/2,0.))


def overlap(a, a_size, b, b_size, *, margin=MARGIN):
    ac,bc = corners(a,a_size),corners(b,b_size)
    for p in (a,b):
        for k in (0,1):
            x=[relative(q,p)[k] for q in ac]
            y=[relative(q,p)[k] for q in bc]
            if max(x)+margin < min(y) or max(y)+margin < min(x):
                return False
    return True


def centered_body_radius(c):
    """Bound a straight translation about the hull centre rather than the wheel pivot."""
    length,width=c['base_size_m']
    offset=c.get('body_center_x_m',0.)
    outline=c.get('empty_footprint_m') or corners((offset,0.,0.),(length,width))
    reserve=max(0.,c['radius_empty_m']-max(math.hypot(x,y) for x,y in outline))
    return max(math.hypot(x-offset,y) for x,y in outline)+reserve


def verify_drop(c, pose, occupied):
    x0,y0,x1,y1 = c['destination_bounds']
    size = c['rack_size_m']
    if not all(x0+MARGIN < x < x1-MARGIN and y0+MARGIN < y < y1-MARGIN
               for x,y in corners(pose,size)):
        raise ValueError('货架整架及余量须在终点区内')
    if any(box_distance(post,pose,size) <= post[2]+MARGIN for post in c['qr_posts']):
        raise ValueError('放置位与二维码立柱间距不足')
    if any(overlap(pose,size,other,size) for other in occupied):
        raise ValueError('放置位与其他落架重叠或余量不足')


class IncompleteCoverage(RuntimeError):
    """The strip has no blocking return but not all thirds were observed."""


def check_corridor(points, origin, meters, size, center_x=0.):
    """Require recent rays through the whole swept strip in each lateral third."""
    half_x,half_y = size[0]/2,size[1]/2
    direction = math.copysign(1.,meters)
    limit = half_x+abs(meters)+MARGIN
    sectors = set()
    ox,oy=(origin[0]-center_x)*direction,origin[1]
    for x,y in points:
        x-=center_x
        if abs(x)<=half_x and abs(y)<=half_y:
            continue
        forward = x*direction
        if 0 < forward <= limit and abs(y)<=half_y+MARGIN:
            raise RuntimeError('定距路径内有雷达回波，已停止')
        if ox < limit < forward:
            cross=oy+(y-oy)*(limit-ox)/(forward-ox)
            if abs(cross)<=half_y+MARGIN:
                sectors.add(-1 if cross < -half_y/3 else 1 if cross > half_y/3 else 0)
    if len(sectors) != 3:
        raise IncompleteCoverage(f'定距方向的通道缺少雷达覆盖（{len(sectors)}/3）')


def rear_strip_cells(grid,base,meters,size,margin):
    """All map cells touching the oriented strip behind the rear bumper."""
    info=grid.info
    o=info.origin
    yaw=math.atan2(2*o.orientation.w*o.orientation.z,1-2*o.orientation.z**2)
    frame=(o.position.x,o.position.y,yaw)
    hx=(-meters+margin)/2
    hy=size[1]/2+margin
    cx,cy=relative(world((-size[0]/2-hx,0.),base),frame)
    c,s=math.cos(base[2]-yaw),math.sin(base[2]-yaw)
    ex,ey=abs(c)*hx+abs(s)*hy,abs(s)*hx+abs(c)*hy
    res=info.resolution
    cell_projection=res*(abs(c)+abs(s))/2
    for j in range(math.floor((cy-ey)/res),math.floor((cy+ey)/res)+1):
        for i in range(math.floor((cx-ex)/res),math.floor((cx+ex)/res)+1):
            x,y=(i+.5)*res,(j+.5)*res
            dx,dy=x-cx,y-cy
            if abs(c*dx+s*dy)<=hx+cell_projection and abs(-s*dx+c*dy)<=hy+cell_projection:
                index=j*info.width+i if 0<=i<info.width and 0<=j<info.height else None
                yield index,world((x,y),frame)


def rear_strip_blockage(grid,base,meters,size,registered=None):
    """Check hard occupancy on the footprint; apply the extra margin once."""
    for margin in (0.,MARGIN):
        for index,point in rear_strip_cells(grid,base,meters,size,margin):
            value=grid.data[index] if index is not None else -1
            if 0<=value<100:continue  # 99 is inflation for a robot centre.
            band='车尾实扫区' if margin==0. else '车尾安全余量'
            if value<0:return f'{band}遇到未知或图外格 {point[0]:.2f}, {point[1]:.2f}'
            if margin and registered and registered(point):continue
            return f'{band}遇到占用格 {point[0]:.2f}, {point[1]:.2f}（代价 {value}）'
    return None


def mapped_rear_strip_clear(grid,base,meters,size):
    return rear_strip_blockage(grid,base,meters,size) is None


def rectangle_gap(a,size_a,b,size_b):
    if overlap(a,size_a,b,size_b,margin=0.):return 0.
    return min(min(box_distance(p,b,size_b) for p in corners(a,size_a)),
               min(box_distance(p,a,size_a) for p in corners(b,size_b)))


def verify_retreat(base, meters, c, occupied, *, allow_departure=False):
    start=rack_pose(base,(c.get('body_center_x_m',0.),0.))
    center=rack_pose(start,(meters/2,0.))
    length,width=c['base_size_m']
    if allow_departure and c.get('empty_footprint_m'):
        width=max(width,2*max(abs(y) for _,y in c['empty_footprint_m']))
    size=(length,width)
    swept=(length+abs(meters),width)
    if any(box_distance(post,center,swept)<=post[2]+MARGIN for post in c['qr_posts']):
        raise ValueError('定距路径靠近二维码立柱')
    for rack in occupied:
        rack_size=c['rack_size_m']
        if not overlap(center,swept,rack,rack_size):continue
        if allow_departure and meters<0:
            initial=rectangle_gap(start,size,rack,rack_size)
            closest=rectangle_gap(center,swept,rack,rack_size)
            final=rectangle_gap(rack_pose(start,(meters,0.)),size,rack,rack_size)
            # Distance between convex bodies is convex along a straight translation:
            # its minimum must be at the start and the endpoint must gain clearance.
            if initial>0 and closest>=initial-1e-9 and final>initial+1e-6:continue
        raise ValueError(f'定距路径靠近登记货架 ({rack[0]:.2f}, {rack[1]:.2f})')


def verify_end_standoff(goal, radius, posts):
    clearance=radius+TRAVEL_MARGIN+NAV_GOAL_TOLERANCE_M
    if any(math.dist(goal[:2],post[:2])<=post[2]+clearance for post in posts):
        raise ValueError('END 停车扫码位与立柱可能摆放的范围间距不足')


class LiveScanBlocked(RuntimeError):
    """A measured return intersects the predicted vehicle clearance."""


def verify_travel(base, velocity, radius, grid, posts, racks, rack_size, *, scan=(), horizon=.6,
                  scan_radius=None, body_size=None, body_center_x=0.):
    """Check the conservative footprint over a short forward projection."""
    vx,vy,omega=velocity
    if not all(math.isfinite(v) for v in (*base,vx,vy,omega)):
        raise RuntimeError('位姿或速度无效，停车')
    speed=math.hypot(vx,vy)
    clearance=radius+TRAVEL_MARGIN
    steps=max(1,math.ceil(speed*horizon/.02))
    scan_limit=clearance
    sweep_pad=0.
    if scan_radius is not None:
        # Bound the distance any corner travels between two sampled poses.
        corner_speed=speed+abs(omega)*math.hypot(abs(body_center_x)+body_size[0]/2,body_size[1]/2)
        steps=max(steps,math.ceil(corner_speed*horizon/.01))
        sweep_pad=corner_speed*horizon/(2*steps)
        scan_limit=scan_radius+speed*horizon/(2*steps)
    info=grid.info
    q=info.origin.orientation
    frame=(info.origin.position.x,info.origin.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
    res=info.resolution
    for step in range(steps+1):
        t=horizon*step/steps
        if abs(omega)<1e-6:
            offset=t*vx,t*vy
        else:
            s,c=math.sin(omega*t)/omega,(1-math.cos(omega*t))/omega
            offset=s*vx-c*vy,c*vx+s*vy
        for hit in scan:
            distance=math.dist(offset,hit)
            if distance<=scan_limit:
                raise LiveScanBlocked(f'实时雷达回波距预计车心 {distance:.3f} m，停车；'
                                   f'当前车体坐标 x={hit[0]:+.3f} m（前为正），y={hit[1]:+.3f} m（左为正），预测 {t:.2f} s')
            if scan_radius is not None and distance<=radius+sweep_pad:
                if box_distance(hit,rack_pose((*offset,omega*t),(body_center_x,0.)),body_size)<=sweep_pad:
                    raise LiveScanBlocked(f'实际车身扫过范围有实时雷达回波，停车；'
                                       f'当前车体坐标 x={hit[0]:+.3f} m（前为正），y={hit[1]:+.3f} m（左为正），预测 {t:.2f} s')
        point=world(offset,base)
        if any(box_distance(point,rack,rack_size)<=clearance for rack in racks):
            raise RuntimeError('预计经过范围与已登记的货架间距不足，停车')
        if any(math.dist(point,post[:2])<=clearance+post[2] for post in posts):
            raise RuntimeError('预计经过范围与二维码立柱间距不足，停车')
        x,y=relative(point,frame)
        for j in range(math.floor((y-clearance)/res),math.floor((y+clearance)/res)+1):
            dy=max(j*res-y,0.,y-(j+1)*res)
            for i in range(math.floor((x-clearance)/res),math.floor((x+clearance)/res)+1):
                dx=max(i*res-x,0.,x-(i+1)*res)
                if dx*dx+dy*dy>clearance*clearance:
                    continue
                if not (0<=i<info.width and 0<=j<info.height and 0<=grid.data[j*info.width+i]<25):
                    raise RuntimeError('预计经过范围靠近围墙或未知地图区域，停车')



def verify_path(points, radius, grid, posts, racks, rack_size):
    """Sweep the rotation-independent outer radius over every path segment."""
    if not points:
        raise ValueError('路径为空')
    previous=points[0]
    for point in points:
        if not all(math.isfinite(v) for v in point) or grid_cell(grid,point) is None:
            raise ValueError('路径点超出有效地图')
        dx,dy=point[0]-previous[0],point[1]-previous[1]
        verify_travel((*previous,0.),(dx,dy,0.),radius,grid,posts,racks,rack_size,horizon=1.)
        previous=point


def grid_cell(grid, point):
    info=grid.info
    q=info.origin.orientation
    x,y=relative(point,(info.origin.position.x,info.origin.position.y,
                        math.atan2(2*q.w*q.z,1-2*q.z*q.z)))
    i,j=math.floor(x/info.resolution),math.floor(y/info.resolution)
    return j*info.width+i if 0<=i<info.width and 0<=j<info.height else None


def in_keepout(point,posts,racks,size,cell_margin):
    return (any(math.dist(point,post[:2])<=post[2]+cell_margin for post in posts)
            or any(box_distance(point,rack,size)<=cell_margin for rack in racks))


def keepout_cells(grid, posts, racks, size, *, margin=MARGIN):
    info=grid.info
    q=info.origin.orientation
    frame=(info.origin.position.x,info.origin.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
    cell_margin=margin+info.resolution/math.sqrt(2)
    indices=[]
    for j in range(info.height):
        for i in range(info.width):
            p=world(((i+.5)*info.resolution,(j+.5)*info.resolution),frame)
            if in_keepout(p,posts,racks,size,cell_margin):
                indices.append(j*info.width+i)
    return indices


def paint_keepout(grid,posts,racks,size,margin):
    # Physical structure is lethal. Its additional planning margin is a soft cost.
    for i in keepout_cells(grid,posts,racks,size,margin=margin):
        if 0<=grid.data[i]<98:grid.data[i]=98
    for i in keepout_cells(grid,posts,racks,size,margin=0.):
        grid.data[i]=100


def local_structure_map(source,posts,racks,size):
    """Rasterize physical objects at 5 mm; the navigation margin stays on the global map."""
    from array import array
    from copy import copy
    import numpy as np
    factor=max(1,round(source.info.resolution/.005))
    grid=copy(source)
    grid.info=copy(source.info)
    grid.info.width*=factor
    grid.info.height*=factor
    grid.info.resolution/=factor
    res=grid.info.resolution
    data=np.asarray(source.data,dtype=np.int8).reshape(source.info.height,source.info.width)
    data=data.repeat(factor,axis=0).repeat(factor,axis=1)
    origin=source.info.origin
    q=origin.orientation
    frame=(origin.position.x,origin.position.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
    X=(np.arange(grid.info.width)+.5)*res
    Y=(np.arange(grid.info.height)[:,None]+.5)*res
    pad=res/math.sqrt(2)
    for x,y,r in posts:
        u,v=relative((x,y),frame)
        data[(X-u)**2+(Y-v)**2<=(r+pad)**2]=100
    for rack in racks:
        u,v=relative(rack[:2],frame)
        c,s=math.cos(rack[2]-frame[2]),math.sin(rack[2]-frame[2])
        dx=np.maximum(abs(c*(X-u)+s*(Y-v))-size[0]/2,0.)
        dy=np.maximum(abs(-s*(X-u)+c*(Y-v))-size[1]/2,0.)
        data[dx*dx+dy*dy<=pad*pad]=100
    grid.data=array('b',data.tobytes())
    return grid
