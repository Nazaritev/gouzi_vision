from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')

    config = LaunchConfiguration('config')
    serial_device = LaunchConfiguration('serial_device')
    enable_camera = LaunchConfiguration('enable_camera')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'limit_bar_duck_test.yaml',
            ]),
        ),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
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
            executable='limit_bar_duck_test_node',
            name='limit_bar_duck_test',
            output='screen',
            parameters=[
                config,
                {
                    'serial_device': serial_device,
                },
            ],
        ),
    ])
