from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the RViz file
    rviz_config_path = os.path.join(
        get_package_share_directory('carver_controller'),
        'rviz',
        'L515_camera.rviz'
    )

    # Add RealSense camera node with the updated path to rs_launch.py
    realsense_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'realsense2_camera', 'rs_launch.py', 
             'enable_rgbd:=true', 'enable_sync:=true', 'align_depth.enable:=true', 
             'enable_color:=true', 'enable_depth:=true', 'publish_tf:=true', 
             'unite_imu_method:=2', 'enable_gyro:=true', 'enable_accel:=true', 
             'enable_infra2:=true', 'enable_infra1:=true'],
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
    
    # Add RViz2
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    # Create the launch description object
    launch_description = LaunchDescription()
    launch_description.add_action(realsense_node)
    launch_description.add_action(imu_filter_node)
    launch_description.add_action(imu_fix_node)
    launch_description.add_action(rviz_node)

    # Return the launch description
    return launch_description
