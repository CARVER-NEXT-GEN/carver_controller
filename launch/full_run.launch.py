#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Paths
    pkg_carver_description = get_package_share_directory('carver_description')
    rviz_path = os.path.join(pkg_carver_description, 'rviz', 'display_merge_lidar.rviz')
    pkg_carver_controller = get_package_share_directory('carver_controller')

    # Nodes and Launch Files

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen'
    )

    # Robot description
    dummy_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_description, 'launch', 'carver.launch.py')
        )
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # LiDAR setup
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_controller, 'launch', 's3_lidar_merge.launch.py')
        )
    )

    # Micro-ROS Agent 1
    micro_ros_agent_1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_1',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '2000000'],
        output='screen'
    )

    # Micro-ROS Agent 2
    micro_ros_agent_2 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_2',
        arguments=['serial', '--dev', '/dev/ttyACM1', '-b', '2000000'],
        output='screen'
    )

    # Launch Description
    launch_description = LaunchDescription()

    # Add Actions
    launch_description.add_action(rviz_node)
    launch_description.add_action(dummy_robot)
    launch_description.add_action(joint_state_publisher_gui)
    launch_description.add_action(lidar_launch)
    # launch_description.add_action(micro_ros_agent_1)
    # launch_description.add_action(micro_ros_agent_2)

    return launch_description
