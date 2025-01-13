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
 
    # Micro-ROS Agent 1
    micro_ros_agent_1 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_1',
        arguments=['serial', '--dev', '/dev/ttyACM1', '-b', '2000000'], #055
        output='screen'
    )

    # Micro-ROS Agent 2
    micro_ros_agent_2 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_2',
        arguments=['serial', '--dev', '/dev/ttyACM2', '-b', '2000000'], #086
        output='screen'
    )
    
    # Micro-ROS Agent 3
    micro_ros_agent_3 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_3',
        arguments=['serial', '--dev', '/dev/ttyACM4', '-b', '2000000'], #amt
        output='screen'
    )
    
    # Micro-ROS Agent 4
    micro_ros_agent_4 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_4',
        arguments=['serial', '--dev', '/dev/ttyACM5', '-b', '2000000'], # interface
        output='screen'
    )
    
    # Micro-ROS Agent 5
    micro_ros_agent_5 = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent_5',
        arguments=['serial', '--dev', '/dev/ttyACM4', '-b', '2000000'],
        output='screen'
    )
    

    # Launch Description
    launch_description = LaunchDescription()
    launch_description.add_action(micro_ros_agent_1)
    launch_description.add_action(micro_ros_agent_2)
    launch_description.add_action(micro_ros_agent_3)
    launch_description.add_action(micro_ros_agent_4)
    # launch_description.add_action(micro_ros_agent_5) 0   3


    return launch_description


def main(args=None):
    try:
        generate_launch_description()
    except KeyboardInterrupt:
        # quit
        sys.exit()


if __name__ == "__main__":
    main()