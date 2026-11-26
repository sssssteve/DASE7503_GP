import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState


def main():
    rclpy.init()
    node = rclpy.create_node('activate_fixed_localization')
    try:
        for name in ('map_server', 'amcl'):
            client = node.create_client(ChangeState, '/'+name+'/change_state')
            if not client.wait_for_service(timeout_sec=12):
                raise RuntimeError(name+' lifecycle service missing')
            for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
                request = ChangeState.Request()
                request.transition.id = transition
                result = client.call_async(request)
                rclpy.spin_until_future_complete(node, result, timeout_sec=8)
                if not result.done() or not result.result().success:
                    raise RuntimeError(name+' failed transition '+str(transition))
            print(name+' active', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
