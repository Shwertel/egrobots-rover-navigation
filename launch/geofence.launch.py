"""Obstacle avoidance plus the TF2 geofence behaviour (Task 2, R4).

The rover cruises forward, avoids obstacles from the LiDAR, and turns back
toward the origin whenever it crosses max_distance_from_origin — a behaviour
driven by its position in the odom frame.

    ros2 launch obstacle_avoider geofence.launch.py
    ros2 service call /start_avoidance std_srvs/srv/Trigger
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
    params_file = os.path.join(pkg_share, 'config', 'geofence_params.yaml')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'rviz': LaunchConfiguration('rviz')}.items()
    )

    geofence_node = Node(
        package='obstacle_avoider',
        executable='obstacle_avoider_node',
        name='obstacle_avoider_node',
        output='screen',
        parameters=[params_file],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        simulation,
        geofence_node,
    ])
