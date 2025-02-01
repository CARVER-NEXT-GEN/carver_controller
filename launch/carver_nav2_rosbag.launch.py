import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

def generate_launch_description():
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    carver_controller_launch_file_dir = os.path.join(get_package_share_directory('carver_controller'), 'launch')
    pkg_carver_description = get_package_share_directory('carver_description')
    pkg_carver_controller = get_package_share_directory('carver_controller')
    config_file = os.path.join(pkg_carver_controller, 'config','mapper_params_online_async.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_dir = os.path.join(get_package_share_directory(
        'carver_controller'), 'maps')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, '/home/carver/Documents/GitHub/CARVER_WS/src/carver_controller/maps/Keepout_1stAutoLoopClosureMapFIBO.yaml'))

    param_dir = os.path.join(get_package_share_directory(
        'carver_controller'), 'config')
    param_file = LaunchConfiguration(
        'params', default=os.path.join(param_dir, 'nav2_params.yaml'))

    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')
    
    #=====================================================================#
    #======================== Generate RVIZ ==============================#
    #=====================================================================#

    rviz_path = os.path.join(pkg_carver_controller,'rviz','nav2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_path],
        output='screen')
    
    # Robot description
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_description, 'launch', 'carver.launch.py')
        )
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    
    #=====================================================================#
    #==================== Generate Robot =================================#
    #=====================================================================#
    
    odometry = Node(
        package='carver_controller',
        executable='ackerman_yaw_rate_odom.py',
        name='ackerman_odom',
        # remappings={("/tf", "/tf_odom")},
        output='screen')
    
    #=====================================================================#
    #====================== Generate IMU Node ============================#
    #=====================================================================#
    messenger = Node(
        package='carver_controller',
        executable='carver_messenger.py',
        name='carver_messenger',
        # remappings={("/tf", "/tf_odom")},
        output='screen')
    
    #=====================================================================#
    #================= Generate Robot localization =======================#
    #=====================================================================#
    
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_carver_controller, 'config','ekf.yaml')]
)
    
    #=====================================================================#
    #===================== Generate RPLidar ==============================#
    #=====================================================================#
    
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_carver_controller, 'launch', 's3_lidar_merge.launch.py')
        )
    )
    
    start_lifecycle_manager_cmd = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_costmap_filters',
    output='screen',
    emulate_tty=True,
    parameters=[{'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['filter_mask_server', 'costmap_filter_info_server']}])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        emulate_tty=True,
        parameters=[param_file])

    start_costmap_filter_info_server_cmd = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        emulate_tty=True,
        parameters=[param_file])
    
    ld = LaunchDescription()

    ld.add_action(joint_state_publisher)
    # ld.add_action(odometry)
    # ld.add_action(messenger)
    ld.add_action(robot_localization_node)
    # ld.add_action(lidar)
    # ld.add_action(start_lifecycle_manager_cmd)
    # ld.add_action(start_map_server_cmd)
    # ld.add_action(start_costmap_filter_info_server_cmd)
    ld.add_action(robot_state_publisher)
    # ld.add_action(rviz)

    # ld.add_action(DeclareLaunchArgument(
    #     'map',
    #     default_value=map_file,
    #     description='Full path to map file to load'))

    # ld.add_action(DeclareLaunchArgument(
    #     'params',
    #     default_value=param_file,
    #     description='Full path to param file to load'))

    # ld.add_action(IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [nav2_launch_file_dir, '/bringup_launch.py']),
    #     launch_arguments={
    #         'map': map_file,
    #         'use_sim_time': use_sim_time,
    #         'params_file': param_file}.items(),
    # ))

    return ld