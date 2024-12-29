# File: master_lidar.launch.py
# Location: carver_controller/launch (or any other desired package/folder)

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    carver_controller_share = get_package_share_directory('carver_controller')

    s3_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(carver_controller_share, 'launch', 's3_lidar.launch.py')
        )
    )

    merge_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(carver_controller_share, 'launch', 'merge_lidar.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(s3_lidar_launch)
    ld.add_action(merge_lidar_launch)

    return ld
