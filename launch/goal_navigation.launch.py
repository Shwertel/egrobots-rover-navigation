"""Obstacle avoidance plus goal navigation — Week 1's behaviour on this rover.

The rover steers toward (goal_x, goal_y) using proportional heading control,
avoiding obstacles on the way, and stops once within goal_tolerance.

    ros2 launch obstacle_avoider goal_navigation.launch.py
    ros2 service call /start_avoidance std_srvs/srv/Trigger

The goal can be overridden at launch:
    ros2 launch obstacle_avoider goal_navigation.launch.py goal_x:=5.0 goal_y:=2.0

Note: this drives to an absolute coordinate, so its accuracy is bounded by
odometry drift, which is significant on a skid-steer platform. See the README.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_avoider')
    params_file = os.path.join(pkg_share, 'config', 'goal_navigation_params.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )

    goal_node = Node(
        package='obstacle_avoider',
        executable='obstacle_avoider_node',
        name='obstacle_avoider_node',
        output='screen',
        parameters=[
            params_file,
            {'goal_x': LaunchConfiguration('goal_x'),
             'goal_y': LaunchConfiguration('goal_y')},
        ],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('goal_x', default_value='10.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        simulation,
        goal_node,
    ])
