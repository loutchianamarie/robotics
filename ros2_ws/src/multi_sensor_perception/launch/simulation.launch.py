"""Gazebo Sim + minimal robot with camera, lidar, IMU (Step 2)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('multi_sensor_perception')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_share, 'worlds', 'msp_empty.sdf')
    xacro_path = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock from Gazebo.',
    )

    robot_description = Command(['xacro ', xacro_path])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(
                    robot_description, value_type=str
                ),
            }
        ],
    )

    # Single string: gz_sim passes this to `gz sim` (see ros_gz_sim gz_sim.launch.py).
    gz_args = f'-r {world_path}'

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
                'use_sim_time': use_sim_time,
            }
        ],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world',
            'msp_world',
            '-name',
            'msp_robot',
            '-topic',
            'robot_description',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.2',
            '-Y',
            '0.0',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    return LaunchDescription(
        [
            declare_use_sim_time,
            gazebo_server,
            robot_state_publisher,
            bridge,
            delayed_spawn,
        ]
    )
