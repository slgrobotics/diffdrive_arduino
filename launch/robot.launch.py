"""Launch the example robot, using fake hardware unless explicitly requested."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('diffdrive_arduino')
    arguments = {
        'use_fake_hardware': 'true',
        'device': '/dev/ttyUSB0',
        'baud_rate': '57600',
        'enc_counts_per_rev': '1920',
        'firmware_loop_rate': '30',
        'timeout': '10',
        'startup_delay_ms': '2000',
    }
    controller_config = os.path.join(share, 'controllers', 'robot_controller_example.yaml')
    xacro_command = [
        FindExecutable(name='xacro'), ' ',
        os.path.join(share, 'description', 'robot.urdf.xacro'),
    ]
    for name in arguments:
        xacro_command += [' ', name, ':=', LaunchConfiguration(name)]
    description = ParameterValue(Command(xacro_command), value_type=str)
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value)
          for name, value in arguments.items()],
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description}], output='screen',
        ),
        Node(
            package='controller_manager', executable='ros2_control_node',
            parameters=[controller_config],
            output='screen',
        ),
        Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster', 'diff_controller',
                       '--controller-manager', '/controller_manager',
                       '--controller-manager-timeout', '30',
                       '--param-file', controller_config],
            output='screen',
        ),
    ])
