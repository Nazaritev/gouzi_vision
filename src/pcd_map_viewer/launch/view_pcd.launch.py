import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_pcd = os.path.expanduser(
        "~/Nazarite_2026-main/Livox_ROS2/src/pgo/src/patches/60.pcd"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_path",
            default_value=default_pcd,
            description="Absolute path to the PCD map file.",
        ),
        DeclareLaunchArgument(
            "topic",
            default_value="/pcd_map",
            description="PointCloud2 topic for the map.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_init",
            description="Frame in which the PCD coordinates are expressed.",
        ),
        DeclareLaunchArgument(
            "fixed_frame",
            default_value="map",
            description="RViz root frame. It is connected to frame_id by an identity static transform.",
        ),
        DeclareLaunchArgument(
            "publish_period",
            default_value="1.0",
            description="Map republish period in seconds.",
        ),
        Node(
            package="pcd_map_viewer",
            executable="pcd_map_publisher",
            name="pcd_map_publisher",
            output="screen",
            parameters=[{
                "pcd_path": LaunchConfiguration("pcd_path"),
                "topic": LaunchConfiguration("topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "publish_period": LaunchConfiguration("publish_period"),
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="pcd_map_frame_publisher",
            arguments=[
                "--frame-id", LaunchConfiguration("fixed_frame"),
                "--child-frame-id", LaunchConfiguration("frame_id"),
            ],
            output="screen",
        ),
    ])
