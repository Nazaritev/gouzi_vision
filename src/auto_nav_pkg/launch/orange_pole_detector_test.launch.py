from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 2026-05-03 01:05 CST:
    # Standalone test launch for the lightweight HSV+depth pole detector.
    # It intentionally does not start YOLO, Point-LIO, nav_executor, or serial
    # bridge so CPU load and detector behavior can be measured in isolation.
    # Rollback: keep using yolo_bringup/all_start.launch.py for YOLO tests.
    auto_share = FindPackageShare('auto_nav_pkg')

    enable_camera = LaunchConfiguration('enable_camera')
    detector_config = LaunchConfiguration('detector_config')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument(
            'detector_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'orange_pole_detector.yaml',
            ]),
        ),
        # 2026-05-03 01:05 CST: D435 at 15 FPS reduces CPU/USB pressure while
        # still giving enough updates for a 1.25m trigger window.
        # Rollback: set both profiles back to 640,480,30 if detection latency
        # is acceptable and smoother debug video is needed.
        DeclareLaunchArgument('color_profile', default_value='640,480,15'),
        DeclareLaunchArgument('depth_profile', default_value='640,480,15'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                ])
            ),
            condition=IfCondition(enable_camera),
            launch_arguments={
                'device_type': 'd435',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'decimation_filter.enable': 'false',
                'pointcloud.enable': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'rgb_camera.profile': color_profile,
                'depth_module.profile': depth_profile,
            }.items(),
        ),

        Node(
            package='auto_nav_pkg',
            executable='orange_pole_detector_node',
            name='orange_pole_detector',
            output='screen',
            parameters=[detector_config],
        ),
    ])
