#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Lidar 1 Configurations
    channel_type_1 = LaunchConfiguration('channel_type_1', default='serial')
    serial_port_1 = LaunchConfiguration('serial_port_1', default='/dev/ttyUSB1')
    serial_baudrate_1 = LaunchConfiguration('serial_baudrate_1', default='1000000')
    frame_id_1 = LaunchConfiguration('frame_id_1', default='laser1')
    inverted_1 = LaunchConfiguration('inverted_1', default='false')
    angle_compensate_1 = LaunchConfiguration('angle_compensate_1', default='true')
    scan_mode_1 = LaunchConfiguration('scan_mode_1', default='DenseBoost')

    # Lidar 2 Configurations
    channel_type_2 = LaunchConfiguration('channel_type_2', default='serial')
    serial_port_2 = LaunchConfiguration('serial_port_2', default='/dev/ttyUSB0')
    serial_baudrate_2 = LaunchConfiguration('serial_baudrate_2', default='1000000')
    frame_id_2 = LaunchConfiguration('frame_id_2', default='laser2')
    inverted_2 = LaunchConfiguration('inverted_2', default='false')
    angle_compensate_2 = LaunchConfiguration('angle_compensate_2', default='true')
    scan_mode_2 = LaunchConfiguration('scan_mode_2', default='DenseBoost')

    return LaunchDescription([
        # Lidar 1 Launch Arguments
        DeclareLaunchArgument(
            'channel_type_1',
            default_value=channel_type_1,
            description='Specifying channel type of lidar 1'),
        DeclareLaunchArgument(
            'serial_port_1',
            default_value=serial_port_1,
            description='Specifying USB port connected to lidar 1'),
        DeclareLaunchArgument(
            'serial_baudrate_1',
            default_value=serial_baudrate_1,
            description='Specifying USB port baudrate connected to lidar 1'),
        DeclareLaunchArgument(
            'frame_id_1',
            default_value=frame_id_1,
            description='Specifying frame_id of lidar 1'),
        DeclareLaunchArgument(
            'inverted_1',
            default_value=inverted_1,
            description='Specifying whether or not to invert scan data for lidar 1'),
        DeclareLaunchArgument(
            'angle_compensate_1',
            default_value=angle_compensate_1,
            description='Specifying whether or not to enable angle compensation for lidar 1'),
        DeclareLaunchArgument(
            'scan_mode_1',
            default_value=scan_mode_1,
            description='Specifying scan mode of lidar 1'),

        # Lidar 2 Launch Arguments
        DeclareLaunchArgument(
            'channel_type_2',
            default_value=channel_type_2,
            description='Specifying channel type of lidar 2'),
        DeclareLaunchArgument(
            'serial_port_2',
            default_value=serial_port_2,
            description='Specifying USB port connected to lidar 2'),
        DeclareLaunchArgument(
            'serial_baudrate_2',
            default_value=serial_baudrate_2,
            description='Specifying USB port baudrate connected to lidar 2'),
        DeclareLaunchArgument(
            'frame_id_2',
            default_value=frame_id_2,
            description='Specifying frame_id of lidar 2'),
        DeclareLaunchArgument(
            'inverted_2',
            default_value=inverted_2,
            description='Specifying whether or not to invert scan data for lidar 2'),
        DeclareLaunchArgument(
            'angle_compensate_2',
            default_value=angle_compensate_2,
            description='Specifying whether or not to enable angle compensation for lidar 2'),
        DeclareLaunchArgument(
            'scan_mode_2',
            default_value=scan_mode_2,
            description='Specifying scan mode of lidar 2'),

        # Lidar 1 Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_1',
            parameters=[{
                'channel_type': channel_type_1,
                'serial_port': serial_port_1,
                'serial_baudrate': serial_baudrate_1,
                'frame_id': frame_id_1,
                'inverted': inverted_1,
                'angle_compensate': angle_compensate_1,
                'scan_mode': scan_mode_1,
                'scan_frequency': 20.0
            }],
            remappings=[('scan', 'lidar_1/scan')],  # Remap 'scan' to 'scan1'
            output='screen'),

        # Lidar 2 Node
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node_2',
            parameters=[{
                'channel_type': channel_type_2,
                'serial_port': serial_port_2,
                'serial_baudrate': serial_baudrate_2,
                'frame_id': frame_id_2,
                'inverted': inverted_2,
                'angle_compensate': angle_compensate_2,
                'scan_mode': scan_mode_2,
                'scan_frequency': 20.0
            }],
            remappings=[('scan', 'lidar_2/scan')],  # Remap 'scan' to 'scan2'
            output='screen'),
    ])
