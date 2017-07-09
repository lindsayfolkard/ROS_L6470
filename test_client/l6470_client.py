import sys

import rclpy

from std_msgs.msg import String
from l6470_msgs.msg import MultiPose


def chatter_callback(msg):
    print('I heard: [%s]' % msg.data)

def pose_callback(msg):
    print('I heard something from the multipose message')
    print('I heard: motor position is [%d]' % msg.motor_states[0].position)

def main(args=None):
    if args is None:
        args = sys.argv

    rclpy.init(args=args)

    node = rclpy.create_node('listener')

    sub = node.create_subscription(String, 'chatter', chatter_callback)
    sub2 = node.create_subscription(MultiPose,'multipose',pose_callback)

    assert sub  # prevent unused warning
    assert sub2

    while rclpy.ok():
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
