import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Path to this package's own share directory (where installed config/launch files live)
    pkg_share = get_package_share_directory('obstacle_avoider')
    params_file = os.path.join(pkg_share, 'config', 'task1_turtlebot_params.yaml')
    # Include TurtleBot3's Gazebo simulation launch file
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
        )
    )

    # Our obstacle avoider node, loading all parameters from the YAML file
    obstacle_avoider_node = Node(
        package='obstacle_avoider',
        executable='obstacle_avoider_node',
        name='obstacle_avoider_node',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        gazebo_launch,
        obstacle_avoider_node,
    ])
