from pathlib import Path
import math
import re
import yaml
from centerline import Centerline
from course_clearance import (
    MARGIN, TRAVEL_MARGIN, NAV_GOAL_TOLERANCE_M, centered_body_radius, world, relative, box_distance, corners, overlap, rack_pose, verify_drop,
    verify_end_standoff, verify_retreat,
)


def numbers(values, name, length):
    if not isinstance(values, list) or len(values) != length or any(
        type(v) not in (int, float) or not math.isfinite(v) for v in values
    ):
        raise ValueError(f'{name} 需要 {length} 个实测数值')
    return [float(v) for v in values]


def qr_post_zones(c):
    posts=c.get('qr_posts')
    if not isinstance(posts,list) or len(posts)!=2:
        raise ValueError('填写 START 和 END 两根立柱的 [x,y,底座半径]')
    positions=[numbers(post,'qr_posts',3) for post in posts]
    shifts=c.get('qr_post_shift_m')
    if shifts is None:
        shifts=[0,0]
    if isinstance(shifts,list):
        shifts=[0 if shift is None else shift for shift in shifts]
    shifts=numbers(shifts,'qr_post_shift_m',2)
    if any(r<=0 for _,_,r in positions) or any(shift<0 for shift in shifts):
        raise ValueError('立柱底座半径须大于零，最大摆放偏移须大于或等于零')
    return [(x,y,r+shift) for (x,y,r),shift in zip(positions,shifts)]


def missing_measurements(c):
    required=('load_center_m',
              'servo_id','lift_up_deg','lift_down_deg','servo_speed_deg_s')
    missing=[key for key in required if c.get(key) is None]
    posts=c.get('qr_posts')
    if not isinstance(posts,list) or len(posts)!=2:
        missing.append('qr_posts')
    else:
        for label,post in zip(('START','END'),posts):
            if post is None:
                missing.append('qr_posts / '+label)
    for i,r in enumerate(c.get('racks') or [],1):
        fields=[]
        if not isinstance(r,dict):
            missing.append(f'P{i} 货架记录')
            continue
        if not r.get('code') or str(r['code']).endswith('_XXXX'):
            fields.append('code（完整二维码）')
        keys=['rack_hint']
        if c.get('drop_policy')=='fixed':
            keys.append('slot')
        fields.extend(k for k in keys if r.get(k) is None)
        if fields:
            missing.append(f'P{i} / '+', '.join(fields))
    return missing


def load(path):
    path = Path(path).resolve()
    c = yaml.safe_load(path.read_text())
    if not isinstance(c.get('map'),str):
        raise ValueError('需填写 map')
    c['map']=(path.parent/c['map']).resolve()
    if not c['map'].is_file():
        raise ValueError(f"地图不存在 {c['map']}")
    map_data = yaml.safe_load(c['map'].read_text())
    if not (c['map'].parent/map_data['image']).is_file():
        raise ValueError('地图图像缺失')
    if c.get('empty_scan_stop_m') is not None and numbers([c['empty_scan_stop_m']],'empty_scan_stop_m',1)[0]<=0:
        raise ValueError('empty_scan_stop_m 必须为正数')
    if numbers([c.get('scan_body_padding_m',0.)],'scan_body_padding_m',1)[0]<0:
        raise ValueError('scan_body_padding_m 不能为负数')
    for window in c.get('scan_echo_windows',[]):
        low,high,rmin,rmax=numbers(window,'scan_echo_windows',4)
        if not (-180<=low<high<=180 and 0<rmin<rmax):
            raise ValueError('固定回波窗口需要有效的角度区间和距离区间')
    if c.get('scan_neighbor_gap_m') is not None:
        gap=numbers([c['scan_neighbor_gap_m']],'scan_neighbor_gap_m',1)[0]
        if not 0<gap<=.10:
            raise ValueError('scan_neighbor_gap_m 须大于零且不超过 0.10 m')
    if c.get('scan_mask_box_m') is not None:
        xmin,ymin,xmax,ymax=numbers(c['scan_mask_box_m'],'scan_mask_box_m',4)
        if xmin>=xmax or ymin>=ymax:
            raise ValueError('scan_mask_box_m 需要 [xmin,ymin,xmax,ymax]')
    missing=missing_measurements(c)
    if missing:
        raise ValueError('待填写的实车配置 '+str(path)+'\n  '+'\n  '.join(missing))
    for key, length in [('laser_mount',6), ('end_approach',3)]:
        c[key] = numbers(c[key], key, length)
    aisle=c.get('centerline')
    if aisle is not None:
        if not isinstance(aisle,dict):
            raise ValueError('centerline 需要填写 y、entry_x 和 end_x')
        y, entry, end=numbers([aisle.get(k) for k in ('y','entry_x','end_x')],'centerline',3)
        if entry==end:
            raise ValueError('中轴的入口与终点侧出入口需要不同的 x 坐标')
        c['centerline']=Centerline(y,entry,end)
    if c.get('start_region') is None:
        c['initial_pose'] = numbers(c.get('initial_pose'), 'initial_pose', 3)
    else:
        c['start_region'] = numbers(c['start_region'], 'start_region', 4)
        x0, y0, x1, y1 = c['start_region']
        if not (0 < x1-x0 <= 2 and 0 < y1-y0 <= 2):
            raise ValueError('start_region 需要按 [xmin,ymin,xmax,ymax] 填写，每边最多 2 m')
    for key in ('radius_empty_m','servo_speed_deg_s'):
        if numbers([c.get(key)],key,1)[0] <= 0:
            raise ValueError('需要正数 '+key)
    if not 1<=c['servo_speed_deg_s']<=180:
        raise ValueError('载货外形或舵机速度不合理')
    for key in ('lift_up_deg','lift_down_deg'):
        if type(c[key]) is not int or not 0<=c[key]<=180:
            raise ValueError('角度无效 '+key)
    if c['lift_up_deg']==c['lift_down_deg'] or type(c['servo_id']) is not int or c['servo_id'] not in (1,2):
        raise ValueError('舵机端口或行程无效')
    for key in ('up_wait_s','down_wait_s'):
        if c.get(key) is not None and numbers([c[key]],key,1)[0]<=0:
            raise ValueError('升降等待时间无效 '+key)
    if c['lift_policy'] not in ('operator','timed'):
        raise ValueError('举升确认策略不明')
    if len(c['racks'])!=4 or {r['code'][:5] for r in c['racks']}!={'RACKA','RACKB','RACKC','RACKD'}:
        raise ValueError('需要四个不同货架')
    for key, n in [('destination_bounds',4),('base_size_m',2),('rack_size_m',2),('load_center_m',2)]:
        c[key]=numbers(c.get(key),key,n)
    x0,y0,x1,y1=c['destination_bounds']
    if not (x0<x1 and y0<y1):
        raise ValueError('终点区边界需要 xmin < xmax 且 ymin < ymax')
    if min(*c['base_size_m'],*c['rack_size_m'])<=0:
        raise ValueError('车体与货架尺寸需要正数')
    if c.get('leg_spacing_m') is not None:
        c['leg_spacing_m']=numbers(c.get('leg_spacing_m'),'leg_spacing_m',2)
    if c.get('leg_spacing_m') is not None and any(not .04 < leg <= top for leg,top in zip(c['leg_spacing_m'],c['rack_size_m'])):
        raise ValueError('架腿中心间距须大于 4 cm 且不超过对应货架尺寸')
    c['qr_posts']=qr_post_zones(c)
    offset=numbers([c.get('body_center_x_m',0.)],'body_center_x_m',1)[0]
    length,width=c['base_size_m']
    if abs(offset)>=length/2:raise ValueError('轮组中心须在车身内')
    turn_radius=math.hypot(length/2+abs(offset),width/2)
    if turn_radius > c['radius_empty_m']:
        raise ValueError('空车外接圆必须覆盖实测车身')
    outline=c.get('empty_footprint_m')
    if outline is not None:
        if len(outline)<3:raise ValueError('空车外形至少需要三个顶点')
        outline=[numbers(point,'empty_footprint_m',2) for point in outline]
        if any(math.hypot(*point)>c['radius_empty_m'] for point in outline):
            raise ValueError('空车旋转半径须覆盖全部轮廓顶点')
        body=corners((offset,0.,0.),c['base_size_m'])
        for a,b in zip(outline,outline[1:]+outline[:1]):
            if a==b or any((b[0]-a[0])*(y-a[1])-(b[1]-a[1])*(x-a[0]) < -1e-9 for x,y in outline+body):
                raise ValueError('空车轮廓须逆时针凸包并覆盖车身')
    needed=max(c['radius_empty_m'],*(math.hypot(x,y) for x,y in corners((*c['load_center_m'],0.),c['rack_size_m'])))
    if c.get('radius_loaded_m') is None:
        c['radius_loaded_m']=math.ceil(needed*1000)/1000
    if numbers([c['radius_loaded_m']],'radius_loaded_m',1)[0]<needed:
        raise ValueError('载货外接圆必须覆盖整架')
    verify_end_standoff(c['end_approach'],c['radius_empty_m'],c['qr_posts'])
    if c.get('retreat_m') is None:
        clear=c['radius_empty_m']+TRAVEL_MARGIN+c['rack_size_m'][0]/2-c['load_center_m'][0]
        c['retreat_m']=math.ceil((clear+.01)*100)/100
    if numbers([c['retreat_m']],'retreat_m',1)[0]<=0:
        raise ValueError('退出距离无效')
    start_heading=c.get('start_heading_rad')
    if start_heading is not None:
        c['start_heading_rad']=numbers([start_heading],'start_heading_rad',1)[0]
    for rack in c['racks']:
        rack['rack_hint']=numbers(rack.get('rack_hint'),rack['code']+'/rack_hint',3)
    for i,rack in enumerate(c['racks']):
        if any(overlap(rack['rack_hint'],c['rack_size_m'],other['rack_hint'],c['rack_size_m'])
               for other in c['racks'][:i]):
            raise ValueError('原场固定货架位置重叠或间距不足')
    policy=c.get('drop_policy','region')
    if policy not in ('region','fixed'):
        raise ValueError('drop_policy 需要 fixed 或 region')
    legacy_targets=None
    occupied=[]
    for rack in c['racks']:
        if not re.fullmatch(r'RACK[A-D]_[A-Za-z0-9]{4}',rack['code']) or rack['code'].endswith('_XXXX'):
            raise ValueError('填写现场的完整码文')
        hint=rack['rack_hint']
        view_radius=centered_body_radius(c)
        depth=c['rack_size_m'][0]/2+view_radius+TRAVEL_MARGIN+2*NAV_GOAL_TOLERANCE_M+offset
        def safe_view(v):
            x,y=world((offset,0.),v)
            limit=view_radius+TRAVEL_MARGIN
            return (min(x,3.02-x,y,2.-y)>limit
                    and all(math.dist((x,y),p[:2])>limit+p[2] for p in c['qr_posts'])
                    and all(box_distance((x,y),h,c['rack_size_m'])>limit for h in
                            [*(r['rack_hint'] for r in c['racks']),*occupied]))
        options=[[*world((-stand,side),hint),hint[2]]
                 for stand in (depth,depth-.08,depth+.12,depth+.32)
                 for side in (0.,.09,-.09,.16,-.16,.22,-.22)]
        options=[v for v in options if safe_view(v)]
        if rack.get('approach') is None and not options:
            raise ValueError('没有安全的雷达观察位 '+rack['code'])
        rack['approach']=numbers(rack.get('approach') or options[0],rack['code']+'/approach',3)
        rack['slot']=None if rack.get('slot') is None else numbers(rack['slot'],rack['code']+'/slot',3)
        if policy=='fixed' and rack['slot'] is None:
            raise ValueError('固定卸货模式缺少 '+rack['code']+' 的 slot')
        views=rack.get('scan_views',[])
        if not isinstance(views,list) or len(views)>2:
            raise ValueError('每架最多两个补扫位')
        rack['scan_views']=[numbers(view,rack['code']+'/scan_views',3) for view in views]
        if not views:
            rack['scan_views']=[v for v in options if math.dist(v[:2],rack['approach'][:2])>.05][:2]
        distance=rack.get('max_insert_m')
        opening=rack.get('opening_m')
        if (distance is None or opening is None) and c.get('alignment'):
            if legacy_targets is None:
                legacy=yaml.safe_load((path.parent/c['alignment']).read_text())
                legacy_targets=legacy['targets']
            calibration=legacy_targets.get(rack['code'],{})
            if distance is None: distance=calibration.get('insert_m')
            if opening is None: opening=calibration.get('opening_m')
        if distance is None:
            distance=-relative(rack['approach'],hint)[0]-c['load_center_m'][0]+NAV_GOAL_TOLERANCE_M
        rack['max_insert_m']=numbers([distance],'max_insert_m',1)[0]
        rack['opening_m']=None if opening is None else numbers([opening],'opening_m',1)[0]
        if distance<=0 or (opening is not None and not c['base_size_m'][1]<opening<=c['rack_size_m'][1]):
            raise ValueError('进架上限或架腿内侧净宽无效 '+rack['code'])
        goals=[(world((offset,0.),v),view_radius) for v in
               [rack['approach'],c['end_approach'],*rack['scan_views']]]
        if rack['slot'] is not None:
            placement=rack_pose(rack['slot'],c['load_center_m'])
            verify_drop(c,placement,occupied)
            verify_retreat(rack['slot'],-c['retreat_m'],c,occupied)
            goals.append((rack['slot'],c['radius_loaded_m']))
        for goal,radius in goals:
            if any(math.dist(goal[:2],post[:2])<=radius+post[2]+MARGIN for post in c['qr_posts']):
                raise ValueError('预靠点与二维码立柱间距不足')
            if any(box_distance(goal,other,c['rack_size_m'])<=radius+2*MARGIN for other in occupied):
                raise ValueError('预靠点与先前落架间距不足')
        if rack['slot'] is not None:
            occupied.append(placement)
    if policy=='fixed':
        if any(box_distance(c['end_approach'],p,c['rack_size_m'])<=c['radius_empty_m']+2*MARGIN for p in occupied):
            raise ValueError('最终 END 扫码位被已放货架占用')
    return c
