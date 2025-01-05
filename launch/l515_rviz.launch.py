from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Get the path to the RViz file
    rviz_config_path = os.path.join(
        get_package_share_directory('carver_controller'),
        'rviz',
        'L515_camera.rviz'
    )
    
    # Get the path to the l515_imu.launch.py file
    l515_imu_launch_path = os.path.join(
        get_package_share_directory('carver_controller'),
        'launch',
        'l515_camera.launch.py'
    )

    # Include the l515_imu.launch.py file
    l515_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(l515_imu_launch_path)
    )
    
    # Add RViz2
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    # Create the launch description object
    launch_description = LaunchDescription()
    launch_description.add_action(l515_imu_launch)
    launch_description.add_action(rviz_node)


    # Return the launch description
    return launch_description
