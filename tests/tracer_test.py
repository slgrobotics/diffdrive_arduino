"""Test compact summaries and command forwarding without starting ROS nodes."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

spec = importlib.util.spec_from_file_location(
    'tracer', Path(__file__).resolve().parents[1] / 'scripts' / 'tracer.py')
tracer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracer)


class TracerTest(unittest.TestCase):
    def setUp(self):
        self.logs = []
        self.sent = []
        self.node = SimpleNamespace(
            latest={}, output_mode='compact',
            get_logger=lambda: SimpleNamespace(info=self.logs.append),
            publisher=SimpleNamespace(publish=self.sent.append),
            get_clock=lambda: SimpleNamespace(
                now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=123))))
        self.node.trace = lambda topic, msg: tracer.Tracer.trace(self.node, topic, msg)

    def test_waiting_then_latest_sample_and_age(self):
        tracer.Tracer.print_summary(self.node)
        self.assertEqual(self.logs[0].count('waiting'), 3)
        self.logs.clear()
        with patch.object(tracer.time, 'monotonic', return_value=10.0):
            self.node.trace('joint_states', JointState(name=['left'], position=[1.0]))
            self.node.trace('joint_states', JointState(name=['left'], position=[2.0]))
            odom = Odometry()
            odom.pose.pose.orientation.w = 1.0
            self.node.trace('diff_controller/odom', odom)
            command = Twist()
            command.linear.x = 0.05
            tracer.Tracer.command(self.node, command)
        self.assertEqual(self.logs, [])
        self.assertEqual(self.sent[0].twist, command)
        self.assertEqual(self.sent[0].header.stamp.sec, 123)
        with patch.object(tracer.time, 'monotonic', return_value=12.0):
            tracer.Tracer.print_summary(self.node)
        summary = self.logs[0]
        self.assertIn('pos=2.000 rad vel=? rad/s', summary)
        self.assertIn('yaw=0.000', summary)
        self.assertIn('vx=0.050', summary)
        self.assertEqual(summary.count('age=2.0s'), 3)
        self.assertEqual(len(summary.strip().splitlines()), 3)

    def test_verbose_logs_immediately(self):
        self.node.output_mode = 'verbose'
        self.node.trace('cmd_vel', Twist())
        self.assertEqual(len(self.logs), 1)
        self.assertIn('linear:', self.logs[0])


if __name__ == '__main__':
    unittest.main()
