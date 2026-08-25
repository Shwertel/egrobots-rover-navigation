"""Shared bring-up: Gazebo, the rover, its controllers, and optionally RViz.

Included by geofence.launch.py and goal_navigation.launch.py, which each add
their own behaviour node on top. Launch this on its own to bring up the robot
with no behaviour attached (useful for manual driving or teleop testing).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_avoider')
    xacro_path = os.path.join(pkg_share, 'urdf', 'egrobots_rover.urdf.xacro')
    world_path = os.path.join(pkg_share, 'worlds', 'egrobots_world.world')
    rviz_config = os.path.join(pkg_share, 'rviz', 'egrobots_rover.rviz')

    use_rviz = LaunchConfiguration('rviz')

    # value_type=str is required: without it launch tries to parse the URDF as
    # YAML, and any "word:" sequence in the XML (including inside a comment)
    # aborts the whole launch after Gazebo has already come up.
    robot_description = ParameterValue(Command(['xacro ', xacro_path]), value_type=str)

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'egrobots_rover', '-z', '0.2'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Launch RViz2 with the coordinate-frame visualisation config'
        ),
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        rviz_node,
    ])
