#!/usr/bin/env python3
"""

Print bench-test feedback and stamp cmd_vel for diff_drive_controller.

"""

import math
import time

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
        self.output_mode = self.declare_parameter('output_mode', 'compact').value
        if self.output_mode not in ('verbose', 'compact'):
            raise ValueError('output_mode must be verbose or compact')
        self.latest = {}
        if self.output_mode == 'compact':
            self.summary_timer = self.create_timer(1.0, self.print_summary)
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
        self.latest[topic] = (msg, time.monotonic())
        if self.output_mode == 'verbose':
            self.get_logger().info(f'{topic}:\n{message_to_yaml(msg)}')

    def print_summary(self):
        lines = ["\n============================================================="]
        now = time.monotonic()
        for topic in ('joint_states', 'diff_controller/odom', 'cmd_vel'):
            if topic not in self.latest:
                lines.append(f'{topic}: waiting')
                continue
            msg, received = self.latest[topic]
            age = now - received
            if topic == 'joint_states':
                def value(values, index):
                    return f'{values[index]:.3f}' if index < len(values) else '?'
                text = ' | '.join(
                    f'{name}: pos={value(msg.position, i)} rad '
                    f'vel={value(msg.velocity, i)} rad/s'
                    for i, name in enumerate(msg.name))
            elif topic == 'diff_controller/odom':
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                v = msg.twist.twist
                text = (f'x={p.x:.3f} y={p.y:.3f} m yaw={yaw:.3f} rad '
                        f'vx={v.linear.x:.3f} m/s wz={v.angular.z:.3f} rad/s')
            else:
                text = f'vx={msg.linear.x:.3f} m/s wz={msg.angular.z:.3f} rad/s (last received)'
            lines.append(f'{topic}: {text} [age={age:.1f}s]')
        self.get_logger().info('\n' + '\n'.join(lines))

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
