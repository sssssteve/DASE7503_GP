"""Verify heading handoff using the production coordinator; no motion publishers."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import ANY, Mock

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from centerline import Centerline, start_passage
try:
    from match import Match,RetreatBlocked
except ModuleNotFoundError as exc:
    if exc.name not in ('rclpy','chassis_interfaces'):raise
    Match=None


@unittest.skipIf(Match is None,'source the ROS and chassis workspaces')
class RackHeadingTest(unittest.TestCase):
    start_config={'rack_size_m':(.14,.38),'radius_empty_m':.195,
                  'racks':[dict(code='RACKB_test',rack_hint=(1.92,.4,math.pi)),
                           dict(code='RACKD_test',rack_hint=(1.81,1.37,math.pi))]}

    def test_start_passage_clear_of_both_racks(self):
        entry,beside,exit=start_passage(self.start_config)
        self.assertAlmostEqual(entry[0],2.245)
        self.assertAlmostEqual(exit[0],1.485)
        self.assertAlmostEqual(entry[1],.845)
        self.assertAlmostEqual(beside[0],1.81)
        self.assertEqual(entry[1:],beside[1:])
        self.assertEqual(entry[1:],exit[1:])

    def test_start_turns_toward_finish_wall_before_translation(self):
        m=Mock(spec=Match)
        m.centerline=Centerline(.955,2.10,1.02)
        m.reconnections=0;m.turn=Mock()
        m.map_base_pose.side_effect=[(2.72,.96,math.radians(-130)),(2.72,.96,math.radians(-179))]
        m.future.side_effect=lambda f,*_:f
        handle=Mock(accepted=True)
        handle.get_result_async.return_value=NS(status=4)
        def accept(req):
            m.follow_centerline.assert_not_called()
            self.assertAlmostEqual(req.target_yaw,math.radians(-50.),places=6)
            return handle
        m.spin_by.side_effect=lambda delta:Match.spin_by(m,delta)
        m.turn.send_goal_async.side_effect=accept
        m.c=self.start_config
        Match.leave_start(m)
        m.turn.send_goal_async.assert_called_once()
        self.assertEqual([c.args[0] for c in m.navigate_transit.call_args_list],list(start_passage(m.c)))

    def test_start_skips_small_error_and_does_not_repeat_blocked_spin(self):
        for error in (23.,49.):
            m=Mock(spec=Match)
            m.centerline=Centerline(.955,2.1,1.02)
            m.map_base_pose.return_value=(2.76,.92,math.pi-math.radians(error))
            m.turn=Mock();m.reconnections=0
            m.spin_by.side_effect=lambda delta:Match.spin_by(m,delta)
            m.future.side_effect=lambda f,*_:f
            m.turn.send_goal_async.return_value.get_result_async.return_value=NS(status=6,result=NS(error_code=703))
            m.c=self.start_config
            Match.leave_start(m)
            self.assertEqual(m.turn.send_goal_async.call_count,int(error>25.))
            self.assertEqual([c.args[0] for c in m.navigate_transit.call_args_list],list(start_passage(m.c)))

    def test_direct_nav_test_skips_aisle_but_keeps_rack_steps_and_slots(self):
        from unittest.mock import call
        for direct in (True,False):
            m=Mock(spec=Match)
            m.centerline=Centerline(.955,2.10,1.02);m.direct_test=direct
            racks=[dict(code='P'+str(i),approach=(1.54,.54,math.pi),slot=(.3,.5+i*.2,0.)) for i in range(1,5)]
            m.c={'racks':racks,'end_approach':(1.02,.90,-2.13)}
            m.standing={r['code']:(1.,1.,math.pi) for r in racks}
            def enter(r):
                return Match.enter_rack(m,r)
            m.enter_rack.side_effect=enter
            m.align.return_value=.32
            Match.nav_recognition_test(m)
            m.leave_start.assert_called_once_with()
            self.assertEqual(m.locate_rack.call_args_list,[call(r,False) for r in racks])
            self.assertEqual(m.move.call_args_list,[call(.32),call(-.32)]*4)
            self.assertEqual(m.navigate_transit.call_args_list,[call(r['slot']) for r in racks]+[call(m.c['end_approach'])])
            m.scan.assert_called_once_with('END');m.lift.assert_not_called()
            self.assertEqual(m.face_rack_at_gate.call_count,0 if direct else 4)
            self.assertEqual(m.follow_centerline.call_count,0 if direct else 12)

    def test_centerline_checks_stopped_position_after_nav_success(self):
        from unittest.mock import call
        m=Mock(spec=Match)
        m.carrying=False
        goal=(2.10,.955,0.)
        m.map_base_pose.side_effect=[(2.03,.75,0.),(2.10,.946,.55),(2.10,.946,.3)]
        Match.navigate_transit(m,goal,centerline=True)
        self.assertEqual(m.navigate.call_args_list,[call(goal,alternate_on_block=True,stop_before=ANY)]*3)
        stop=m.navigate.call_args.kwargs['stop_before']
        m.map_base_pose.side_effect=[(2.10,.946,math.radians(23)),(2.10,.80,0.)]
        self.assertTrue(stop())
        self.assertFalse(stop())
        self.assertEqual(m.pause.call_count,2)

    def test_dead_end_checks_sides_backs_up_and_replans_same_goal(self):
        from unittest.mock import call
        goal=(2.1,.955,math.pi)
        m=Mock(spec=Match)
        m.carrying=False
        m.last={'odom':NS(pose=NS(pose=NS(position=NS(x=0.,y=0.))))}
        m.navigate.side_effect=[False,False,True]
        m.try_side_detour.return_value=False
        m.map_base_pose.side_effect=[(2.4,.96,math.pi),(2.4,.96,math.pi),goal]
        m.checked_backup.side_effect=[False,True]  # Rear blocked, then clear.
        Match.navigate_transit(m,goal,centerline=True)
        trace=[k for k,*_ in m.mock_calls if k in ('navigate','try_side_detour','checked_backup')]
        self.assertEqual(trace,['navigate','try_side_detour','checked_backup']*2+['navigate'])
        self.assertEqual(m.checked_backup.call_args_list,[call(-.15,'导航脱困',include_target=True)]*2)
        self.assertTrue(all(args[0]==goal for args,_ in m.navigate.call_args_list))
        m.scan.assert_not_called();m.lift.assert_not_called()

    def test_arrival_braking_is_not_dead_end_recovery(self):
        goal=(2.10,.955,math.pi)
        moving=(2.075,.915,math.pi)
        for stop_y,calls in ((.896,1),(.855,2)):
            m=Mock(spec=Match)
            m.carrying=False
            m.map_base_pose.side_effect=[moving,(2.069,stop_y,math.pi-math.radians(13.1)),goal,goal]
            def navigate(target,*,alternate_on_block,stop_before):
                self.assertTrue(stop_before())
                return False  # The coordinator deliberately cancelled FollowPath.
            m.navigate.side_effect=navigate
            Match.navigate_transit(m,goal,centerline=True)
            self.assertEqual(m.navigate.call_count,calls)
            m.try_side_detour.assert_not_called()
            m.checked_backup.assert_not_called()

    def test_loaded_noncenterline_retries_detours_and_checked_reverses(self):
        m=Mock(spec=Match);m.carrying=True
        m.last={'odom':NS(pose=NS(pose=NS(position=NS(x=0.,y=0.))))}
        goal=(.5,.6,math.pi)
        m.navigate.side_effect=[False,False,False,True]
        m.checked_backup.return_value=True
        m.try_side_detour.return_value=False
        m.map_base_pose.return_value=(1.5,.6,math.pi)
        Match.navigate_transit(m,goal)
        self.assertEqual(m.try_side_detour.call_count,3)
        self.assertEqual(m.checked_backup.call_count,3)
        self.assertEqual(m.navigate.call_count,4)
        m.lift.assert_not_called()

    def test_dead_end_checks_outer_edge_of_carried_rack(self):
        from course_clearance import mapped_rear_strip_clear
        m=Mock(spec=Match)
        m.carrying=True
        m.travel_envelope.side_effect=lambda:Match.travel_envelope(m)
        m.c={'base_size_m':(.28,.19),'rack_size_m':(.14,.38),'load_center_m':(-.035,0.),'qr_posts':[]}
        m.standing={};m.deposited=[]
        m.map_base_pose.return_value=(1.,1.,0.)
        grid=NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.02,
                origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*10000)
        grid.data[58*100+39]=100
        m.fresh.return_value=True
        m.last={'costmap':grid}
        m.backup_client=Mock()
        self.assertTrue(mapped_rear_strip_clear(grid,(1.,1.,0.),-.15,(.28,.19)))
        self.assertFalse(Match.checked_backup(m,-.15,'导航脱困',include_target=True))
        m.backup_client.send_goal_async.assert_not_called()

    def test_pose_correction_without_motion_does_not_count_as_detour(self):
        m=Mock(spec=Match);m.fixed=True
        m.map_base_pose.side_effect=[(2.,1.,math.pi),(2.07,1.,math.pi)]
        self.assertFalse(Match.try_side_detour(m,(1.54,.955,math.pi)))
        m.navigate.assert_not_called()

    def test_exit_coverage_gap_uses_map_but_obstacle_or_unknown_waits(self):
        m=Mock(spec=Match)
        m.c={'base_size_m':(.28,.19),'body_center_x_m':.03,'qr_posts':[],'rack_size_m':(.14,.38)}
        grid=NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.02,
                origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*10000)
        m.last={'scan':object(),'costmap':grid};m.fresh.return_value=True
        m.scan_points.return_value=([(1.,0.)],(.13,0.))  # 0/3 rear sectors, live front return
        base=(1.,1.,0.)
        Match.check_exit_clearance(m,base,-.33,[])
        for cost in (-1,100):
            grid.data[50*100+39]=cost
            with self.assertRaises(RetreatBlocked):Match.check_exit_clearance(m,base,-.33,[])
        grid.data[50*100+39]=0
        m.scan_points.return_value=([(-.23,0.)],(.13,0.))
        with self.assertRaises(RetreatBlocked):Match.check_exit_clearance(m,base,-.33,[])
        m.scan_points.return_value=([],(.13,0.))
        with self.assertRaises(RetreatBlocked):Match.check_exit_clearance(m,base,-.33,[])

    def test_interrupted_exit_cancels_and_uses_only_remaining_distance(self):
        m=Mock(spec=Match);m.reconnections=0;m.relative_id=17
        m.map_base_pose.side_effect=[(1.,1.,0.),(.88,1.,0.)]
        m.relative_leg.side_effect=[RetreatBlocked('有障碍'),True]
        Match.move(m,-.33)
        self.assertEqual(m.send.call_args.args[0],'/chassis/cancel')
        self.assertEqual(m.send.call_args.args[1].data,17)
        dist=[args[0] for args,_ in m.relative_leg.call_args_list]
        self.assertAlmostEqual(dist[0],-.33);self.assertAlmostEqual(dist[1],-.21)

    def test_side_choice_uses_scan_and_rear_corner_clearance(self):
        m=Mock(spec=Match)
        m.strafe_peaks=None
        m.map_base_pose.return_value=(1.,1.,0.)
        m.c={'base_size_m':(.28,.19)}
        m.last={'scan':object(),'costmap':NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.02,
                  origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*10000)}
        m.fresh.return_value=True
        m.carrying=False;m.travel_envelope.side_effect=lambda:Match.travel_envelope(m)
        rays=[(math.cos(math.radians(i)),math.sin(math.radians(i))) for i in range(360)]
        m.fixed=True
        for points,angle,goal,travel in (
                (rays,math.pi/2,(1.25,1.,0.),.25),
                (rays+[(-.10,-.13)],math.pi/2,(1.25,1.,0.),.25),
                (rays+[(0.,.27)],math.pi/2,(1.25,1.,0.),.12),
                (rays+[(0.,.23)],-math.pi/2,(1.25,1.,0.),.25)):
            m.navigate.reset_mock()
            m.scan_points.return_value=(points,(.10,0.))
            m.navigate.return_value=True
            self.assertTrue(Match.try_side_detour(m,goal))
            m.navigate.assert_called_once_with((1+travel*math.cos(angle),1+travel*math.sin(angle),0.),
                                               alternate_on_block=True,strafe=1 if angle>0 else -1)
        m.navigate.reset_mock()
        m.scan_points.return_value=(rays,(.10,0.))
        from geometry_msgs.msg import Pose
        odom=Pose();odom.orientation.z=math.sin(math.radians(160.6)/2);odom.orientation.w=math.cos(math.radians(160.6)/2)
        m.last['odom']=NS(pose=NS(pose=odom))
        origin=(1.851,1.024,math.radians(160.6))
        m.map_base_pose.return_value=origin
        # Both lateral strips are blocked; the oblique path towards the waypoint is clear.
        m.scan_points.return_value=(rays+[(-.11,.23),(-.11,-.23)],(.10,0.))
        def advance(via,**kw):
            end=Pose();end.position.x=via[0]-origin[0];end.position.y=via[1]-origin[1];m.last['odom'].pose.pose=end
            m.map_base_pose.return_value=via
            return True
        m.navigate.side_effect=advance
        self.assertTrue(Match.try_side_detour(m,(1.54,.955,math.pi)))
        via=m.navigate.call_args.args[0]
        self.assertAlmostEqual(math.dist(via[:2],(1.851,1.024)),.12)
        self.assertAlmostEqual((via[1]-1.024)/(via[0]-1.851),(.955-1.024)/(1.54-1.851))
        self.assertLess(via[0],1.851)
        self.assertAlmostEqual(via[2],math.radians(160.6))
        m.navigate.reset_mock()
        m.scan_points.return_value=([],(.10,0.))
        self.assertFalse(Match.try_side_detour(m,(1.6,1.,0.)))
        m.navigate.assert_not_called()

    def test_strafe_vy_keeps_signed_peak_zero_and_missing_distinct(self):
        m=Mock(spec=Match)
        m.strafe_peaks=None
        Match.record_strafe_vy(m,'/cmd_vel_nav',.24)
        m.strafe_peaks={}
        for v in (-.06,-.14,.03,0.,math.nan):
            Match.record_strafe_vy(m,'/cmd_vel_nav',v)
        Match.record_strafe_vy(m,'/cmd_vel',0.)
        self.assertEqual(m.strafe_peaks,{'/cmd_vel_nav':-.14,'/cmd_vel':0.})

    def test_rack_gate_uses_opening_heading_even_when_arriving_diagonally(self):
        m=Mock(spec=Match)
        m.centerline=Centerline(y=.885,entry_x=2.1,end_x=1.02)
        m.map_base_pose.return_value=(1.54,.885,math.radians(-178))
        rack={'code':'P1','approach':(1.54,.54,math.pi),'rack_hint':(1.13,.54,math.pi)}
        self.assertEqual(Match.face_rack_at_gate(m,rack),(1.54,.885,math.pi))
        m.follow_centerline.assert_called_once_with((1.54,.885,math.pi))
        m.scan.assert_not_called()

    def test_lateral_recovery_selects_controller_and_verifies_displacement(self):
        from geometry_msgs.msg import Pose
        from builtin_interfaces.msg import Time
        for sign,observed,success in ((1,.085,True),(-1,-.083,True),(1,0.,False)):
            m=Mock(spec=Match)
            m.fixed=True;m.carrying=False;m.reconnections=0;m.nav=Mock()
            m.get_clock.return_value.now.return_value.to_msg.return_value=Time()
            odom=NS(pose=NS(pose=Pose()))
            odom.pose.pose.orientation.w=1.
            grid=NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.02,
                    origin=NS(position=NS(x=-1.,y=-1.),orientation=NS(w=1.,z=0.))),data=[0]*10000)
            m.last={'odom':odom,'scan':object(),'costmap':grid}
            m.travel_envelope.return_value=((.28,.19),.03)
            m.fresh.return_value=True
            m.scan_points.return_value=([(math.cos(i*.02),math.sin(i*.02)) for i in range(315)],(.13,0.))
            m.strafe_request.side_effect=lambda goal,side:Match.strafe_request(m,goal,side)
            m.map_base_pose.side_effect=[(0.,0.,0.),(0.,observed,0.)]
            m.future.side_effect=lambda f,*_:f
            handle=Mock(accepted=True)
            def result():
                odom.pose.pose=Pose()
                odom.pose.pose.position.y=observed
                return NS(status=4,result=NS(error_code=0))
            handle.get_result_async.side_effect=result
            m.nav.send_goal_async.return_value=handle
            self.assertEqual(Match.navigate(m,(0.,sign*.12,0.),alternate_on_block=True,strafe=sign),success)
            m.checked_route.assert_not_called()
            req=m.nav.send_goal_async.call_args.args[0]
            self.assertEqual(req.controller_id,'StrafeLeft' if sign==1 else 'StrafeRight')
            self.assertAlmostEqual(req.path.poses[-1].pose.position.y,sign*.12)
            self.assertTrue(all(p.pose.position.x==0 and p.pose.orientation.w==1 for p in req.path.poses))

    def test_impossible_rack_fit_is_discarded_and_next_view_is_tried(self):
        from unittest.mock import patch
        import match
        rack=dict(code='P1',rack_hint=(1.13,.54,math.pi),approach=(1.54,.54,math.pi),
                  max_insert_m=.5)
        for overlap_other in (True,False):
            m=Mock(spec=Match);m.centerline=None
            m.standing={'P1':rack['rack_hint']};m.target_legs=[]
            m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'load_center_m':(0.,0.),'leg_spacing_m':(.11,.35)}
            m.last={'scan':NS(header=NS(stamp=0)),'map':object()}
            m.scan_points.return_value=([],(0.,0.))
            m.other_racks.return_value=[rack['rack_hint']] if overlap_other else []
            m.navigate.return_value=m.prepare_rack_scan.return_value=m.scan.return_value=True
            with patch.object(match,'fit_rack',return_value=(rack['rack_hint'],[(0,0)]*4)),patch.object(match,'grid_cell',return_value=None):
                self.assertFalse(Match.survey_views(m,rack,[rack['approach'],(1.54,.63,math.pi)]))
            self.assertEqual(m.navigate.call_count,2)
            m.scan.assert_not_called()
            m.prepare_rack_scan.assert_not_called()
            m.publish_keepout.assert_not_called()
            self.assertEqual(m.target_legs,[])

    def test_offset_camera_retries_to_body_left_before_reverse(self):
        for yaw in (0.,math.pi/2,math.pi,-math.pi/2):
            for moved,read_after_left in ((True,True),(True,False),(False,False)):
                with self.subTest(yaw=yaw,moved=moved,read=read_after_left):
                    m=Mock(spec=Match);m.fixed=True;m.centerline=None
                    base=(1.,1.,yaw)
                    rack=dict(code='P1',rack_hint=(1.3,1.,yaw),approach=base)
                    m.standing={'P1':rack['rack_hint']}
                    m.last={'scan':NS(header=NS(stamp=0))}
                    m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19)}
                    m.scan_points.return_value=([],(0.,0.))
                    m.prepare_rack_scan.return_value=True
                    m.map_base_pose.return_value=base
                    m.navigate.side_effect=[True,moved]
                    m.scan.side_effect=[False,read_after_left,True] if moved else [False,True]
                    m.backup_for_qr.return_value=True
                    Match.survey_views(m,rack,[base])
                    goal=m.navigate.call_args.args[0]
                    self.assertAlmostEqual(goal[0],base[0]-.08*math.sin(yaw))
                    self.assertAlmostEqual(goal[1],base[1]+.08*math.cos(yaw))
                    self.assertEqual(goal[2],yaw)
                    self.assertEqual(m.navigate.call_args.kwargs,dict(alternate_on_block=True,strafe=1))
                    sequence=[name for name,*_ in m.mock_calls if name in ('scan','backup_for_qr','navigate')]
                    expected=['navigate','scan','navigate']
                    if moved:expected.append('scan')
                    if not read_after_left:expected.extend(['backup_for_qr','scan'])
                    self.assertEqual(sequence,expected)

    def test_unread_rack_code_only_visits_exterior_views(self):
        from unittest.mock import call
        m=Mock(spec=Match);m.fixed=False
        rack=dict(code='P1',rack_hint=(1.13,.54,math.pi),approach=(1.54,.54,math.pi))
        views=[rack['approach'],(1.54,.63,math.pi)]
        m.standing={'P1':rack['rack_hint']}
        m.last={'scan':NS(header=NS(stamp=0))}
        m.scan_points.return_value=([],(0.,0.))
        m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19)}
        m.prepare_rack_scan.return_value=True
        m.navigate.return_value=True
        m.centerline=Centerline(.955,2.1,1.02)
        m.scan.return_value=False
        m.backup_for_qr.side_effect=[False,True,False]
        self.assertFalse(Match.survey_views(m,rack,views))
        sequence=[entry for entry in m.mock_calls if entry[0] in ('navigate','scan','backup_for_qr')]
        self.assertEqual([entry[0] for entry in sequence],[
            'navigate','scan','backup_for_qr', 'navigate','scan','backup_for_qr','scan',
            'navigate','scan','backup_for_qr'])
        far=sequence[3].args[0]
        self.assertAlmostEqual(far[0],1.62)
        self.assertAlmostEqual(far[1],.78)
        self.assertAlmostEqual(far[2],math.atan2(.54-.78,1.13-1.62))
        self.assertEqual(sequence[1],call.scan('P1',try_another_view=True,quick=True))
        self.assertEqual(sequence[6],call.scan('P1',try_another_view=True))
        self.assertEqual(views,[rack['approach'],(1.54,.63,math.pi)])
        m.align.assert_not_called()
        m.move.assert_not_called()

    def test_logged_p1_close_pose_reverses_straight_before_scanning(self):
        m=Mock(spec=Match)
        pose=(1.13,.54,math.pi)
        rack=dict(code='P1',rack_hint=pose,approach=(1.54,.54,math.pi))
        m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'radius_empty_m':.17,'qr_posts':[]}
        m.rack_scan_backoff.side_effect=lambda r,p:Match.rack_scan_backoff(m,r,p)
        m.map_base_pose.return_value=(1.40,.64,math.pi)
        def backup(r,dx):
            self.assertAlmostEqual(dx,-.14)
            m.map_base_pose.return_value=(1.54,.64,math.pi)
            return True
        m.backup_for_qr.side_effect=backup
        self.assertTrue(Match.prepare_rack_scan(m,rack,pose,False))
        self.assertTrue(Match.prepare_rack_scan(m,rack,pose,True))
        m.backup_for_qr.assert_called_once()

    def test_p1_skew_recovers_before_scan(self):
        # At 24 cm the circular turning envelope touches P1; back away before the turn.
        for offset,angle,first in ((.24,140.,'backup_for_qr'),(.30,-140.,'spin_by')):
            with self.subTest(offset=offset,angle=angle):
                m=Mock(spec=Match)
                hint=(1.13,.54,math.pi)
                rack=dict(code='P1',rack_hint=hint,approach=(1.54,.54,math.pi))
                base=[hint[0]+offset,hint[1],math.radians(angle)]
                m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'radius_empty_m':.17}
                m.turn=Mock();m.reconnections=0;m.fresh.return_value=True
                m.map_base_pose.side_effect=lambda:tuple(base)
                m.rack_scan_backoff.side_effect=lambda r,p:Match.rack_scan_backoff(m,r,p)
                m.recover_rack_heading.side_effect=lambda r,p:Match.recover_rack_heading(m,r,p)
                def reverse(r,dx):
                    base[0]+=dx*math.cos(base[2]);base[1]+=dx*math.sin(base[2])
                    return True
                def spin(delta):
                    self.assertAlmostEqual(delta,math.radians(40 if angle>0 else -40))
                    base[2]+=delta
                    return True
                m.backup_for_qr.side_effect=reverse;m.spin_by.side_effect=spin
                self.assertIsNone(m.rack_scan_backoff(rack,hint))
                self.assertTrue(Match.prepare_rack_scan(m,rack,hint,False))
                motions=[method for method,*_ in m.mock_calls if method in ('spin_by','backup_for_qr')]
                self.assertEqual(motions[0],first)
                self.assertEqual(motions.count('spin_by'),1)
                self.assertEqual(m.rack_scan_backoff(rack,hint),0.)
                m.move.assert_not_called()

    def test_skew_without_clearance_does_not_scan_or_claim_another_view(self):
        m=Mock(spec=Match)
        hint=(1.13,.54,math.pi)
        rack=dict(code='P1',rack_hint=hint,approach=(1.54,.54,math.pi),scan_views=[])
        m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'radius_empty_m':.17}
        m.turn=Mock();m.reconnections=0;m.fresh.return_value=True
        m.map_base_pose.return_value=(1.37,.54,math.radians(140))
        m.rack_scan_backoff.side_effect=lambda r,p:Match.rack_scan_backoff(m,r,p)
        m.recover_rack_heading.side_effect=lambda r,p:Match.recover_rack_heading(m,r,p)
        m.backup_for_qr.return_value=False
        def survey(r,views):
            return Match.prepare_rack_scan(m,r,hint,False)
        m.survey_views.side_effect=survey
        m.pause.side_effect=KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            Match.locate_rack(m,rack)
        m.backup_for_qr.assert_called_once_with(rack,-.08)
        m.spin_by.assert_not_called();m.scan.assert_not_called();m.move.assert_not_called()
        m.pause.assert_called_once_with(2.)

    def test_spin_ack_without_heading_change_does_not_unlock_scan(self):
        m=Mock(spec=Match)
        pose=(1.13,.54,math.pi)
        m.map_base_pose.return_value=(1.43,.54,math.radians(140))
        m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'radius_empty_m':.17}
        m.fresh.return_value=True;m.turn=Mock();m.reconnections=0;m.spin_by.return_value=True
        m.backup_for_qr.return_value=False
        self.assertFalse(Match.recover_rack_heading(m,{'code':'P1'},pose))
        m.spin_by.assert_called_once()
        m.scan.assert_not_called()

    def test_qr_reverse_with_partial_scan_requires_map_and_measured_displacement(self):
        from geometry_msgs.msg import Pose
        m=Mock(spec=Match)
        m.map_base_pose.return_value=(1.,1.,0.)
        m.carrying=False
        m.travel_envelope.side_effect=lambda:Match.travel_envelope(m)
        m.checked_backup.side_effect=lambda *args,**kw:Match.checked_backup(m,*args,**kw)
        m.c={'rack_size_m':(.14,.38),'base_size_m':(.28,.19),'qr_posts':[]}
        grid=NS(header=NS(frame_id='map'),info=NS(width=100,height=100,resolution=.02,
                origin=NS(position=NS(x=0.,y=0.),orientation=NS(w=1.,z=0.))),data=[0]*10000)
        m.last={'scan':object(),'odom':NS(pose=NS(pose=Pose())),'costmap':grid}
        m.other_racks.return_value=[]
        m.scan_points.return_value=([(-.8,.16)],(.1,0.))  # Only one rear sector.
        m.fresh.return_value=True
        m.backup_client=Mock()
        m.reconnections=0
        m.future.side_effect=lambda f,*_:f
        handle=Mock(accepted=True)
        def completed():
            p=Pose();p.position.x=-.08
            m.last['odom'].pose.pose=p
            return NS(status=4,result=NS(error_code=0))
        handle.get_result_async.side_effect=completed
        m.backup_client.send_goal_async.return_value=handle
        rack={'code':'P1'}
        self.assertTrue(Match.backup_for_qr(m,rack,-.08))
        req=m.backup_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(req.target.x,-.08)
        self.assertEqual(req.target.y,0.)
        self.assertFalse(Match.backup_for_qr(m,rack,-.08))  # ACK without new displacement.
        m.backup_client.send_goal_async.reset_mock()
        m.scan_points.return_value=([(-.19,0.)],(.1,0.))
        self.assertFalse(Match.backup_for_qr(m,rack,-.08))
        m.scan_points.return_value=([],(.1,0.))
        for cost in (100,-1):
            grid.data[50*100+40]=cost
            self.assertFalse(Match.backup_for_qr(m,rack,-.08))
        m.backup_client.send_goal_async.assert_not_called()

    def test_missed_and_wrong_code_release_view_then_correct_code_passes(self):
        m=Mock(spec=Match)
        m.reconnections=0;m.last={}
        m.get_clock.return_value.now.return_value.nanoseconds=1
        states=iter([('TIMEOUT',''),('SUCCEEDED','P2'),('SUCCEEDED','P1')])
        import json
        def respond(topic,message):
            request=json.loads(message.data)
            state,decoded=next(states)
            m.last['vision']=dict(request_id=request['id'],expected=request['expected'],
                                  state=state,text=decoded,stamp_nanosec=1)
        m.send.side_effect=respond
        m.wait.side_effect=lambda done,*_:self.assertTrue(done())
        self.assertFalse(Match.scan(m,'P1',try_another_view=True))
        self.assertFalse(Match.scan(m,'P1',try_another_view=True))
        self.assertTrue(Match.scan(m,'P1',try_another_view=True))
        self.assertEqual(m.send.call_count,3)

    def test_detour_endpoint_keeps_planned_xy_and_gets_requested_yaw(self):
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path as RosPath
        m=Mock(spec=Match)
        m.c={'path_mode':'straight'}
        m.reconnections=0
        m.future.side_effect=lambda f,*unused:f
        m.planner=Mock()
        m.map_base_pose.return_value=(1.8,.9,-2.269)
        def pose(x,y,yaw):
            p=PoseStamped()
            p.header.frame_id='map'
            p.pose.position.x=x;p.pose.position.y=y
            p.pose.orientation.z=math.sin(yaw/2);p.pose.orientation.w=math.cos(yaw/2)
            return p
        path=RosPath();path.header.frame_id='map'
        path.poses=[pose(1.8,.9,-2.269),pose(1.72,.83,-1.),pose(1.55,.87,-2.55)]
        goal=pose(1.54,.885,math.pi)
        handle=Mock(accepted=True)
        handle.get_result_async.return_value=NS(status=4,result=NS(path=path))
        m.planner.send_goal_async.return_value=handle
        req=Match.checked_route(m,goal)
        self.assertEqual([(p.pose.position.x,p.pose.position.y) for p in req.path.poses],[(1.8,.9),(1.72,.83),(1.55,.87)])
        self.assertEqual(req.path.poses[-1].pose.orientation,goal.pose.orientation)
        self.assertEqual(req.path.poses[1].pose.orientation,pose(1.72,.83,-1.).pose.orientation)

    def test_104_releases_survey_view_after_stopping(self):
        from builtin_interfaces.msg import Time
        from nav2_msgs.action import FollowPath
        from unittest.mock import call
        m=Mock(spec=Match)
        m.fixed=True;m.reconnections=0;m.nav=Mock()
        m.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        m.future.side_effect=lambda f,*_:f
        blocked=Mock(accepted=True)
        blocked.get_result_async.return_value=NS(status=6,result=NS(error_code=FollowPath.Result.PATIENCE_EXCEEDED))
        m.nav.send_goal_async.return_value=blocked
        self.assertFalse(Match.navigate(m,(1.54,.54,math.pi),alternate_on_block=True))
        m.nav.send_goal_async.assert_called_once()
        self.assertEqual(m.mode.call_args_list,[call('nav'),call('stop')])

    def test_close_while_nav_is_following_cancels_before_reverse(self):
        from builtin_interfaces.msg import Time
        from unittest.mock import call
        m=Mock(spec=Match)
        m.fixed=True;m.reconnections=0;m.nav=Mock()
        m.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        handle=Mock(accepted=True)
        pending=handle.get_result_async.return_value
        pending.done.return_value=False
        m.nav.send_goal_async.return_value=handle
        m.future.side_effect=lambda f,*_:f
        m.wait.side_effect=lambda ready,*_:self.assertTrue(ready())
        guard=Mock(side_effect=[False,True])
        self.assertFalse(Match.navigate(m,(1.54,.54,math.pi),alternate_on_block=True,stop_before=guard))
        handle.cancel_goal_async.assert_called_once()
        self.assertEqual(m.mode.call_args_list,[call('nav'),call('stop')])
        m.move.assert_not_called()

    def test_survey_visits_alternates_then_keeps_same_rack(self):
        from unittest.mock import call
        m=Mock(spec=Match)
        rack=dict(code='P1',rack_hint=(1.13,.54,math.pi),approach=(1.54,.54,math.pi),
                  scan_views=[(1.54,.63,math.pi),(1.54,.45,math.pi)])
        views=[rack['approach'],*rack['scan_views']]
        m.standing={'P1':rack['rack_hint']}
        m.map_base_pose.return_value=(1.97,1.09,2.87)
        m.rack_scan_backoff.return_value=0.
        m.navigate.return_value=False
        self.assertFalse(Match.survey_views(m,rack,views))
        self.assertEqual(m.navigate.call_args_list,[call((*v[:2],math.atan2(.54-v[1],1.13-v[0])),alternate_on_block=True,stop_before=ANY) for v in views])
        m.scan.assert_not_called()
        m.try_side_detour.assert_called_once()
        m.survey_views.side_effect=[False,True]
        Match.locate_rack(m,rack)
        self.assertEqual(m.survey_views.call_args_list,[call(rack,views),call(rack,views)])
        m.mode.assert_called_once_with('stop')
        m.move.assert_not_called()
