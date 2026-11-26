"""Exercise mission recovery with synthetic feedback; no ROS node or motor publisher."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, call

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
try:
    from match import Match
except ModuleNotFoundError as exc:
    if exc.name not in ('rclpy','chassis_interfaces'): raise
    Match=None


@unittest.skipIf(Match is None,'source the ROS and chassis workspaces')
class DeviceReconnectTest(unittest.TestCase):
    def waiting_node(self):
        m=Mock(spec=Match)
        m.wait_links=True
        m.phase_state={'state':'insert'}
        m.carrying=False
        m.lift_target=None
        m.last_good_odom=None
        m.nav_goal=None
        m.relative_id=19
        m.running=False
        m.paused_seconds=0.
        m.reconnections=0
        m.fresh.return_value=True
        m.sensor_fault.return_value=None
        cs=NS(action_state=0,vx_mps=0.,vy_mps=0.,wz_rad_s=0.,x_m=.4,y_m=.3,yaw_rad=.1)
        m.last={'scan':NS(header=NS(stamp=0)),'chassis':cs,'gate':{'mode':'stop'}}
        m.poll.side_effect=lambda:setattr(m.last['scan'].header,'stamp',m.last['scan'].header.stamp+1)
        m.device_gap.side_effect=['scan 数据过期','scan 数据过期',None,None,None]
        return m

    def test_gap_keeps_same_rack_and_stops_until_three_new_scans(self):
        m=self.waiting_node()
        Match.wait_for_devices(m)
        self.assertEqual(m.poll.call_count,4)
        self.assertEqual(m.reconnections,1)
        self.assertEqual(m.phase_state,{'state':'insert'})
        self.assertEqual(m.lease,'stop')
        commands=[(topic,msg.data) for (topic,msg),_ in m.send.call_args_list]
        self.assertIn(('/chassis/cancel',19),commands)
        self.assertTrue(all(data in ('stop',19) for _,data in commands))

    def test_scan_expiring_between_poll_and_motion_check_pauses(self):
        m=Mock(spec=Match);m.wait_links=True;m.running=True;m.fresh.return_value=False
        m.device_gap.return_value='scan 采样过期'
        Match.spin(m)
        self.assertEqual(m.wait_for_devices.call_args_list,[call(),call('scan 采样过期')])
        m.send.assert_not_called()
        m.device_gap.return_value=None
        m.sensor_fault.return_value='ESP32 故障或急停'
        with self.assertRaisesRegex(RuntimeError,'急停'):
            Match.spin(m)

    def test_gui_stop_or_ctrl_c_can_interrupt_wait(self):
        m=self.waiting_node()
        m.poll.side_effect=KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):Match.wait_for_devices(m)
        self.assertEqual(m.reconnections,0)

    def test_reverse_and_forward_resume_only_remaining_distance(self):
        for sign in (-1,1):
            m=Mock(spec=Match)
            m.reconnections=1
            m.map_base_pose.side_effect=[(1.,.3,math.pi/2),(1.,.3+sign*.12,math.pi/2)]
            m.relative_leg.side_effect=[False,True]
            Match.move(m,sign*.30)
            lengths=[args[0] for args,_ in m.relative_leg.call_args_list]
            self.assertAlmostEqual(lengths[0],sign*.30)
            self.assertAlmostEqual(lengths[1],sign*.18)

    def test_completed_displacement_not_sent_again(self):
        m=Mock(spec=Match)
        m.reconnections=1
        m.map_base_pose.side_effect=[(0.,0.,0.),(.3,0.,0.)]
        m.relative_leg.return_value=False
        Match.move(m,.3)
        self.assertEqual(m.relative_leg.call_count,1)

    def test_interrupted_navigation_replans_the_original_goal(self):
        m=Mock(spec=Match)
        m.fixed=True
        m.reconnections=0
        from builtin_interfaces.msg import Time
        m.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        m.future.side_effect=lambda future,*_:future
        old, new=Mock(),Mock()
        old.accepted=new.accepted=True
        m.nav=Mock()
        m.nav.send_goal_async.side_effect=[old,new]
        def lost():
            m.reconnections+=1
            return NS(status=5)
        old.get_result_async.side_effect=lost
        new.get_result_async.return_value=NS(status=4)
        Match.navigate(m,(1.2,.7,.4))
        self.assertEqual(m.checked_route.call_count,2)
        goals=[a[0].pose.position for a,_ in m.checked_route.call_args_list]
        self.assertEqual([(p.x,p.y) for p in goals],[(1.2,.7)]*2)
        self.assertIsNone(m.nav_goal)

    def test_nav2_obstruction_replans_same_target_without_direct_drive_checks(self):
        from builtin_interfaces.msg import Time
        from nav2_msgs.action import FollowPath
        m=Mock(spec=Match)
        m.fixed=True;m.reconnections=0;m.nav=Mock()
        m.get_clock.return_value.now.return_value.to_msg.return_value=Time()
        m.future.side_effect=lambda f,*_:f
        m.checked_route.side_effect=[None,'path A','path B']
        blocked,ok=Mock(accepted=True),Mock(accepted=True)
        blocked.get_result_async.return_value=NS(status=6,result=NS(error_code=FollowPath.Result.NO_VALID_CONTROL))
        ok.get_result_async.return_value=NS(status=4)
        m.nav.send_goal_async.side_effect=[blocked,ok]
        Match.navigate(m,(1.2,.7,.4))
        self.assertEqual(m.nav.send_goal_async.call_count,2)
        self.assertEqual(m.checked_route.call_count,3)
        self.assertEqual(m.mode.call_args_list,[call('nav'),call('stop'),call('nav'),call('stop')])
        m.check_alignment_travel.assert_not_called()

    def test_qr_request_after_reconnect_uses_new_token_same_code(self):
        import json
        m=Mock(spec=Match)
        m.reconnections=0
        m.last={'vision':{}}
        m.get_clock.return_value.now.return_value.nanoseconds=100
        requests=[]
        def send(topic,message):
            requests.append(json.loads(message.data))
        def wait(fn,*_):
            target=requests[-1]
            if len(requests)==1:m.reconnections+=1
            else:
                m.last['vision']=dict(state='SUCCEEDED',request_id=target['id'],expected='START',
                                      text='START',stamp_sec=0,stamp_nanosec=100)
            self.assertTrue(fn())
        m.send.side_effect=send
        m.wait.side_effect=wait
        Match.scan(m,'START')
        self.assertEqual([r['expected'] for r in requests],['START','START'])
        self.assertNotEqual(requests[0]['id'],requests[1]['id'])
