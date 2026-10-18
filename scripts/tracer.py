#!/usr/bin/env python3
"""

Print bench-test feedback and stamp cmd_vel for diff_drive_controller.

"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_yaml
from sensor_msgs.msg import JointState


class Tracer(Node):
    def __init__(self):
        super().__init__('tracer')
        self.publisher = self.create_publisher(TwistStamped, 'diff_controller/cmd_vel', 10)
        self.command_subscription = self.create_subscription(Twist, 'cmd_vel', self.command, 10)
        self.feedback_subscriptions = [
            self.create_subscription(msg_type, topic,
                                     lambda msg, topic=topic: self.trace(topic, msg),
                                     qos_profile_sensor_data)
            for msg_type, topic in [(JointState, 'joint_states'),
                                    (Odometry, 'diff_controller/odom')]
        ]
        self.get_logger().info('Tracing joint_states, diff_controller/odom and cmd_vel')

    def trace(self, topic, msg):
        self.get_logger().info(f'{topic}:\n{message_to_yaml(msg)}')

    def command(self, msg):
        self.trace('cmd_vel', msg)
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.twist = msg
        self.publisher.publish(command)


def main():
    rclpy.init()
    node = Tracer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
