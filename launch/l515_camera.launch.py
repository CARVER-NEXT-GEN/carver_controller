from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Add RealSense camera node with the updated path to rs_launch.py
    realsense_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 
             'enable_rgbd:=true', 'enable_sync:=true', 'align_depth.enable:=true', 
             'enable_color:=true', 'enable_depth:=true', 'publish_tf:=true', 
             'unite_imu_method:=2', 'enable_gyro:=true', 'enable_accel:=true', 
             'enable_infra2:=true', 'enable_infra1:=true', 'pointcloud.enable:=true'],
        output='screen'
    )
    
    # Add IMU Filter Madgwick node
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        parameters=[
            {'use_mag': False},
            {'publish_tf': False},
            {'world_frame': 'enu'},
            {'fixed_frame': 'camera_link'}
        ],
        remappings=[
            ('/imu/data_raw', '/camera/imu'),
            ('/imu/data', '/rtabmap/imu')
        ]
    )
    
    # Add custom IMU fix frame node
    imu_fix_node = Node(
        package='carver_controller',
        executable='fix_imu_frame.py',
        name='fix_imu_frame_node',
        parameters=[
            {'frame_id': 'camera_link'},
            {'child_frame_id': 'camera_imu_optical_frame'}
        ]
    )
    
    # Add RTAB-Map node
    rtabmap_node = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'rtabmap_launch', 'rtabmap.launch.py',
            'rtabmap_args:=--delete_db_on_start --Optimizer/GravitySigma 0.3',
            'depth_topic:=/camera/aligned_depth_to_color/image_raw',
            'rgb_topic:=/camera/color/image_raw',
            'camera_info_topic:=/camera/color/camera_info',
            'approx_sync:=false',
            'wait_imu_to_init:=true',
            'imu_topic:=/imu_l515/data',
            'frame_id:=camera_link',
            'depth_camera_info_topic:=/camera/depth/camera_info',
            'qos:=0'
        ],
        output='screen'
    )

    # Create the launch description object
    launch_description = LaunchDescription()
    launch_description.add_action(rtabmap_node)
    launch_description.add_action(realsense_node)
    launch_description.add_action(imu_filter_node)
    launch_description.add_action(imu_fix_node)

    # Return the launch description
    return launch_description
