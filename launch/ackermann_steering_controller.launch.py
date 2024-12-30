from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="false",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("carver_description"), "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    
#     robot_description_content = Command(
#     [
#         PathJoinSubstitution([FindExecutable(name="xacro")]),
#         " /home/carver/Documents/GitHub/CARVER_WS/src/carver_description/urdf/robot.urdf.xacro",
#     ]
# )
    
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("carver_controller"),
            "config",
            "ackermann_steering_controller_params.yaml",
        ]
    )
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("carver_description"),
            "rviz",
            "display.rviz",
        ]
    )

    # Nodes for control and visualization
    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("/ackermann_steering_controller/tf_odometry", "/tf"),
        ],
        condition=IfCondition(remap_odometry_tf),
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[],
        condition=UnlessCondition(remap_odometry_tf),
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--param-file", robot_controllers],
    )

    # Delay RViz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of joint_state_broadcaster after `ackermann_controller_spawner`
    delay_joint_state_broadcaster_after_ackermann_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ackermann_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        control_node_remapped,
        robot_state_pub_node,
        ackermann_controller_spawner,
        delay_joint_state_broadcaster_after_ackermann_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
