import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

def generate_launch_description():
 
     # use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_dir = os.path.join(get_package_share_directory("carver_controller"), "config")
    # config_file = os.path.join(config_dir, 'mapper_params_online_sync.yaml')

    rviz_config_dir = os.path.join(
        get_package_share_directory("carver_controller"), "rviz"
    )
    rviz_config_file = os.path.join(rviz_config_dir, "mapping.rviz")
    
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

 
    messenger = Node(
    package='carver_controller',
    executable='carver_messenger.py',
    name='carver_messenger',
    # remappings={("/tf", "/tf_odom")},
    output='screen')
    
    odometry = Node(
        package='carver_controller',
        executable='ackerman_yaw_rate_odom.py',
        name='ackerman_odom',
        # remappings={("/tf", "/tf_odom")},
        output='screen')
    
    motor = Node(
    package='carver_controller',
    executable='carver_odrive_manual_steering.py',
    name='carver_motor',
    # remappings={("/tf", "/tf_odom")},
    output='screen')
    

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(messenger)
    launch_description.add_action(odometry)
    launch_description.add_action(rviz)


    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()