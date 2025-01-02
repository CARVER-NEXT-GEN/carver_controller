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
    
    pkg_carver_controller = get_package_share_directory('carver_controller')

    # LiDAR Merger RVIZ setup
    lidar_merger_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_controller, 'launch', 'lidar_merger_rviz.launch.py')
        )
    )
    
    micro_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_controller, 'launch', 'micro_ros.launch.py')
        )
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    config_file = os.path.join(pkg_carver_controller, 'config','mapper_params_online_sync.yaml')
    
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
    
    slam_toolbox =  Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='sync_slam_toolbox_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, config_file])
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_carver_controller, 'config','ekf.yaml')]
)
    

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(lidar_merger_rviz)
    # launch_description.add_action(micro_ros)
    launch_description.add_action(messenger)
    # launch_description.add_action(odometry)
    # launch_description.add_action(slam_toolbox)
    # launch_description.add_action(robot_localization_node)


    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()