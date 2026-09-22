from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')
    apriltag_share = FindPackageShare('apriltag_ros')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    auto_share,
                    'launch',
                    'obstacle_race_visual_supervisor.launch.py',
                ])
            ),
            launch_arguments={
                'apriltag_config': PathJoinSubstitution([
                    apriltag_share,
                    'cfg',
                    'tags_36h11_limit_bar_right_test.yaml',
                ]),
                'supervisor_overlay_config': PathJoinSubstitution([
                    auto_share,
                    'config',
                    'obstacle_race_supervisor_limit_bar_tag_test_overlay.yaml',
                ]),
                'enable_apriltag': 'true',
            }.items(),
        ),
    ])
