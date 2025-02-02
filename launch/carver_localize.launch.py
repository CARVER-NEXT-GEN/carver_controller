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
    
    # ==============================================================================
    param_dir = os.path.join(get_package_share_directory(
        'carver_controller'), 'config')
    param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'navigation_param.yaml'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    map_dir = os.path.join(get_package_share_directory(
        'carver_controller'), 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, '1stAutoLoopClosureMapFIBO.yaml')) #gensurv_outdoor1
    # ==============================================================================


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
    executable='carver_invkin.py',
    name='carver_motor',
    # remappings={("/tf", "/tf_odom")},
    output='screen')
    
    slam_toolbox =  Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='async_slam_toolbox_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, config_file]
    )
    
    
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_carver_controller, 'config','ekf.yaml')]
    )
    
    slam_toolbox_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'slam_toolbox', 'async_slam_toolbox_node',
            '--ros-args',
            '-p', f'use_sim_time:=false',
            '--params-file', config_file
        ],
        output='screen'
    )
    
    #==============================================================================
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_file_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': param_file
        }.items(),
    )

    #==============================================================================

    pkg = get_package_share_directory('carver_controller')
    rviz_path = os.path.join(pkg,'rviz','navigation.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(lidar_merger_rviz)
    launch_description.add_action(micro_ros)
    launch_description.add_action(messenger)
    launch_description.add_action(motor)
    launch_description.add_action(odometry)
    launch_description.add_action(robot_localization_node)
    launch_description.add_action(rviz)
    launch_description.add_action(bringup_launch)

    
    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()