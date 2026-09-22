from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("quadruped_step_mapper"), "config", "cmd_vel_to_serial.yaml"]
        ),
        description="Path to the serial bridge parameter file.",
    )

    node = Node(
        package="quadruped_step_mapper",
        executable="cmd_vel_to_serial_node",
        name="cmd_vel_to_serial",
        output="screen",
        parameters=[LaunchConfiguration("config")],
    )

    return LaunchDescription([config_arg, node])
