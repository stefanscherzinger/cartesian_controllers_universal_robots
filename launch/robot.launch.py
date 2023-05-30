# -----------------------------------------------------------------------------
# \file    robot.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2023/05/30
#
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    this_pkg = FindPackageShare("cartesian_controllers_universal_robots")

    # Declare arguments
    arg_robot_ip = DeclareLaunchArgument(
        "robot_ip", default_value="192.168.1.9", description="The robot's IP address"
    )
    declared_args = [arg_robot_ip]

    # Robot description
    description_file = PathJoinSubstitution([this_pkg, "urdf", "setup.urdf.xacro"])
    robot_ip = LaunchConfiguration("robot_ip")
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            description_file,
            " ",
            "robot_ip:=",
            robot_ip,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot control
    robot_controllers = PathJoinSubstitution([this_pkg, "config", "controller_manager.yaml"])
    control_node = Node(
        package="ur_robot_driver",
        executable="ur_ros2_control_node",
        output="screen",
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('force_torque_sensor_broadcaster/wrench', 'ft_sensor_wrench'),
            ],
        parameters=[robot_description, robot_controllers],
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    # Active controllers
    active_list = [
            "joint_state_broadcaster",
            "force_torque_sensor_broadcaster",
            "scaled_joint_trajectory_controller"
            ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    # Inactive controllers
    inactive_list = [
            "cartesian_compliance_controller",
            "cartesian_force_controller",
            "cartesian_motion_controller",
            "motion_control_handle"
            ]
    inactive_spawners = [controller_spawner(controller, "--inactive") for controller in inactive_list]


    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Visualization
    rviz_config = PathJoinSubstitution([this_pkg, "etc", "setup.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    # Nodes to start
    nodes = [rviz, control_node, robot_state_publisher] + active_spawners + inactive_spawners

    return LaunchDescription(declared_args + nodes)
