import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    pkg_share = get_package_share_directory('obstacle_avoider')
    xacro_path = os.path.join(pkg_share, 'urdf', 'egrobots_rover.urdf.xacro')
    params_file = os.path.join(pkg_share, 'config', 'task2_egrobots_rover_params.yaml')

    robot_description = Command(['xacro ', xacro_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    world_path = os.path.join(pkg_share, 'worlds', 'egrobots_world.world')
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
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

    obstacle_avoider_node = Node(
        package='obstacle_avoider',
        executable='obstacle_avoider_node',
        name='obstacle_avoider_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
        ]
    )
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        obstacle_avoider_node,
    ])
