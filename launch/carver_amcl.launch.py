import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
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
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    config_file = os.path.join(pkg_carver_controller, 'config','mapper_params_online_async.yaml')
    #config_file = '/home/carver/Documents/GitHub/CARVER_WS/src/carver_controller/config/mapper_params_online.yaml'
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
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
    
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_carver_controller, 'config','ekf.yaml')]
)
    amcl_node = Node(
       package='nav2_amcl',
       executable='amcl',
       name='amcl_node',
       output='screen',
       parameters=[os.path.join(pkg_carver_controller, 'config','amcl_params.yaml')]
)
    
    map_file = os.path.join(pkg_carver_controller, 'maps', 'Keepout_1stAutoLoopClosureMapFIBO.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': "false",
                'yaml_filename': map_file  # e.g. "/home/user/maps/office_map.yaml"
            }
        ]
    )

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(lidar_merger_rviz)
    # launch_description.add_action(micro_ros)
    launch_description.add_action(messenger)
    # launch_description.add_action(motor)
    launch_description.add_action(odometry)
    launch_description.add_action(robot_localization_node)
    launch_description.add_action(amcl_node)
    launch_description.add_action(map_server_node)


    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()