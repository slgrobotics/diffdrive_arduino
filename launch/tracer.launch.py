"""

Standalone hardware bench test for ROS 2 Lyrical.

Tracer prints wheel feedback, odometry, and cmd_vel.
Serial DEBUG logs print all completed command/reply exchanges.
Launch includes the real hardware plugin and diff-drive controller.
YAML filesin "config" directory configure hardware and wheel geometry.


This launch file is intended to be run on the robot.
It will start the ROS 2 nodes required to communicate with the robot and control it.

Raise the wheels for initial testing.

Launch it:
    ros2 launch diffdrive_arduino tracer.launch.py

  or:

    ros2 launch diffdrive_arduino tracer.launch.py output_mode:=verbose

Send wheel commands:
    ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.05}, angular: {z: 0.0}}'

"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context):
    share = Path(get_package_share_directory('diffdrive_arduino'))
    output_mode = LaunchConfiguration('output_mode').perform(context)
    if output_mode not in ('verbose', 'compact'):
        raise ValueError('output_mode must be verbose or compact')
    description = xacro.process_file(
        str(share / 'description' / 'tracer.urdf.xacro'),
        mappings={'hardware_config': LaunchConfiguration('hardware_config').perform(context),
                  'output_mode': output_mode}
    ).toxml()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description}], output='screen'),

        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[LaunchConfiguration('controllers')], output='screen',
             arguments=['--ros-args', '--log-level',
                        'DiffDriveArduino:=' + ('debug' if output_mode == 'verbose' else 'info')]
        ),

        Node(package='controller_manager', executable='spawner',
             arguments=['joint_state_broadcaster', 'diff_controller',
                        '--controller-manager-timeout', '60'], output='screen'),

        Node(package='diffdrive_arduino', executable='tracer.py', output='screen',
             parameters=[{'output_mode': output_mode}]),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('diffdrive_arduino'))
    return LaunchDescription([
        DeclareLaunchArgument('hardware_config', default_value=str(share / 'config' / 'tracer_hardware.yaml')),
        DeclareLaunchArgument('controllers', default_value=str(share / 'config' / 'tracer_controllers.yaml')),
        DeclareLaunchArgument('output_mode', default_value='compact',
                              description='verbose: every message; compact: latest data once per second'),
        OpaqueFunction(function=launch_setup),
    ])
