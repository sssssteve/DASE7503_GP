import os
import json
import sys
from pathlib import Path
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,GroupAction,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap,Node
from ament_index_python.packages import get_package_share_directory

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE))
from config import load

def generate_launch_description():
    c=load(os.environ['DASE_MATCH_CONFIG'])
    gui = os.environ.get('DASE_GUI_EXTERNAL_LOCALIZATION') == '1'
    # Use the installed Jazzy distribution's defaults, with scoped overrides.
    root=Path(get_package_share_directory('nav2_bringup'))
    params=yaml.safe_load((root/'params/nav2_params.yaml').read_text())
    amcl=params['amcl']['ros__parameters']
    region=c.get('start_region') is not None
    amcl.update(robot_model_type='nav2_amcl::OmniMotionModel',base_frame_id='base_link',
                scan_topic='/scan_body_filtered',update_min_d=.06,update_min_a=.10,set_initial_pose=not region)
    if not region:
        x,y,yaw=c['initial_pose']
        amcl['initial_pose']=dict(x=x,y=y,z=0.,yaw=yaw)
    controller=params['controller_server']['ros__parameters']
    controller.update(min_y_velocity_threshold=.001)
    controller['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',
        required_movement_radius=.12,required_movement_angle=.20)
    follow=controller['FollowPath']
    if follow['plugin']!='nav2_mppi_controller::MPPIController':
        raise ValueError('核对已安装 Nav2 的控制器配置')
    empty_test=os.environ.get('DASE_EMPTY_NAV_TEST')=='1'
    forward=.25 if empty_test else .18
    reverse=.22 if empty_test else .12
    accel=.60 if empty_test else .35
    decel=.75 if empty_test else .35
    follow.update(motion_model='Omni',vx_max=forward,vy_max=forward,vx_min=-reverse,wz_max=.8,visualize=False,batch_size=1000)
    if c.get('empty_footprint_m'):
        follow['CostCritic']['consider_footprint']=True
    # Mecanum base: allow signed translation along the path while discouraging spin.
    follow['critics'] = [name for name in follow['critics'] if name not in ('PreferForwardCritic', 'PathAngleCritic', 'TwirlingCritic')]
    follow['critics'].append('TwirlingCritic')
    follow['TwirlingCritic'] = {'enabled': True, 'cost_power': 1, 'cost_weight': 15.0}
    follow['PathAlignCritic']['use_path_orientations'] = False
    if c.get('drop_policy')=='fixed':
        for name,low,high in (('StrafeLeft',0.,.14),('StrafeRight',-.14,0.)):
            controller['controller_plugins'].append(name)
            controller[name]=dict(plugin='dwb_core::DWBLocalPlanner',
                min_vel_x=0.,max_vel_x=0.,min_vel_y=low,max_vel_y=high,max_vel_theta=0.,
                min_speed_xy=0.,max_speed_xy=.14,min_speed_theta=0.,
                acc_lim_x=accel,decel_lim_x=-decel,acc_lim_y=accel,decel_lim_y=-decel,
                acc_lim_theta=1.,decel_lim_theta=-1.,vx_samples=1,vy_samples=15,vtheta_samples=1,
                sim_time=.8,linear_granularity=.01,
                critics=['ObstacleFootprint','GoalDist'],**{'ObstacleFootprint.scale':.002,'GoalDist.scale':2.})
    # Leave room for the planner endpoint offset inside the mission's arrival tolerance.
    controller['general_goal_checker'].update(xy_goal_tolerance=.04,yaw_goal_tolerance=.12,stateful=False)
    params['velocity_smoother']['ros__parameters'].update(max_velocity=[forward,forward,.8],min_velocity=[-reverse,-forward,-.8],
        max_accel=[accel,accel,1.],max_decel=[-decel,-decel,-1.])
    for name in ('local_costmap','global_costmap'):
        cost=params[name][name]['ros__parameters']
        if name=='global_costmap':
            cost['always_send_full_costmap']=True
        cost.update(robot_radius=float(c['radius_empty_m']),resolution=.02,footprint_padding=.005)
        if c.get('empty_footprint_m'):
            cost['footprint']=json.dumps(c['empty_footprint_m'])
        local_grid=name=='local_costmap'
        if local_grid:
            cost.update(resolution=.005,width=2,height=2,footprint_padding=0.)
        cost['static_layer'].update(map_topic='/match/local_structure_map' if local_grid else '/match/keepout_map',map_subscribe_transient_local=True,
                                    trinary_costmap=False)
        if 'static_layer' not in cost['plugins']:
            cost['plugins'].insert(0,'static_layer')
        live_layer='voxel_layer' if name=='local_costmap' else 'obstacle_layer'
        cost[live_layer]['scan'].update(topic='/scan_body_filtered',marking=True,clearing=True)
        cost['inflation_layer'].update(inflation_radius=max(.28,c['radius_loaded_m']+.06),cost_scaling_factor=8.)
    planners=params['planner_server']['ros__parameters']
    planners['GridBased'].update(allow_unknown=False,tolerance=.02)
    path_mode = c.get('path_mode', 'navfn')
    if path_mode not in ('navfn', 'straight'):
        raise ValueError('path_mode 需要填写 navfn 或 straight')
    if path_mode == 'straight':
        planners['planner_plugins']=['GridBased','AnyAngle']
        planners['AnyAngle']=dict(plugin='nav2_theta_star_planner::ThetaStarPlanner',allow_unknown=False,
                                  w_euc_cost=1.0,w_traversal_cost=2.0,use_final_approach_orientation=False)
        params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(HERE/'straight_or_detour.xml')
    monitor=params['collision_monitor']['ros__parameters']
    monitor.update(base_frame_id='base_link',cmd_vel_out_topic='/nav_cmd_vel',source_timeout=.4)
    monitor['scan'].update(topic='/scan_body_filtered',min_height=-.05,max_height=1.)
    monitor['FootprintApproach'].update(min_points=2,footprint_topic='/global_costmap/published_footprint')
    if empty_test:
        monitor['FootprintApproach']['time_before_collision']=.9
    # Preserve file throughout launch and clean it when ros2 launch exits.
    import atexit
    with tempfile.NamedTemporaryFile(mode='w',delete=False,suffix='.yaml') as f:
        yaml.safe_dump(params,f);paramfile=f.name
    atexit.register(Path(paramfile).unlink,missing_ok=True)
    def include(name,**extra):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch'/name)),launch_arguments={
            'autostart':'True','use_sim_time':'False','use_composition':'False','params_file':paramfile,**extra}.items())
    # Real camera, Agent, ESP32 and lidar must be started first. No mock sources.
    transform=[]
    for name,value in zip(('x','y','z','roll','pitch','yaw'),c['laser_mount']):
        transform.extend(['--'+name,str(value)])
    navigation = GroupAction([SetRemap(src='docking_server:cmd_vel',dst='/unused_docking_cmd_vel'),include('navigation_launch.py')])
    competitors = [
        ExecuteProcess(cmd=['/usr/bin/python3',str(HERE/'body_scan.py'),os.environ['DASE_MATCH_CONFIG']]),
        ExecuteProcess(cmd=['/usr/bin/python3',str(HERE/'control.py')]),
        ExecuteProcess(cmd=['/usr/bin/python3',str(HERE.parent/'simulation/logistics/selective_scan.py'),'--ros-args','-p','output_dir:='+str(Path.home()/'dase7503_game_evidence')]),
    ]
    if gui:
        return LaunchDescription([*competitors,navigation])
    return LaunchDescription([
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=transform+['--frame-id','base_link','--child-frame-id','laser_frame']),
        *competitors,
        include('localization_launch.py',map=str(c['map'])),
        navigation,
    ])
