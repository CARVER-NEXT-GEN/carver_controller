from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import sys
import os

def generate_launch_description():
    
    # sick_scan_pkg_prefix = get_package_share_directory('sick_scan_xd')
    # launchfile = os.path.basename(__file__)[:-3] # convert "<lidar_name>.launch.py" to "<lidar_name>.launch"
    # launch_file_path = os.path.join(sick_scan_pkg_prefix, 'launch/' + launchfile) # 'launch/sick_ldmrs.launch')
    # node_arguments=[launch_file_path]
    
    config = os.path.join(
        get_package_share_directory('carver_controller'),
        'config',
        'sick_point2scan.yaml'
    )
    
    # # append optional commandline arguments in name:=value syntax
    # for arg in sys.argv:
    #     if len(arg.split(":=")) == 2:
    #         node_arguments.append(arg)
    
    # # Sick scanner node with parameters and remapping
    # sick_scan_launch = Node(
    #     package='sick_scan_xd',
    #     executable='sick_generic_caller',
    #     name='sick_scan',
    #     output='screen',
    #     parameters=[{
    #         'hostname': '192.168.1.46',
    #         'port': '2111',  # Port as a string
    #     }],
    #     remappings=[
    #         ('/cloud', '/scanner/cloud')
    #     ]
    # )
    
    sick_scan_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'sick_scan_xd', 'sick_ldmrs.launch.py', 
             'hostname:=192.168.1.46', 'port:=2111', '/cloud:=/scanner/cloud', ],
        output='screen'
    )
    
    # Add IMU Filter Madgwick node
    point2scan = Node(
        name='pointcloud_to_laserscan_sick',
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        parameters=[config],
        remappings=[
            ('/cloud_in', '/scanner/cloud')
        ]
    )
    
    # Add RViz2
    # rviz_node = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz_config_path],
    #     output='screen'
    # )

    # Create the launch description object
    launch_description = LaunchDescription()
    launch_description.add_action(sick_scan_launch)
    launch_description.add_action(point2scan)
    # launch_description.add_action(rviz_node)

    # Return the launch description
    return launch_description
