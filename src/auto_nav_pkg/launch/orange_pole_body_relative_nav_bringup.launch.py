from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 2026-05-04 14:11 CST:
    # Separate HSV body-relative route test. The base bringup stays unchanged;
    # this launch only adds a second orange_pole_relative_nav parameter file so
    # waypoints after point 2 can be expressed relative to the robot's current
    # body heading.
    auto_share = FindPackageShare('auto_nav_pkg')
    step_share = FindPackageShare('quadruped_step_mapper')

    system_config = LaunchConfiguration('system_config')
    body_relative_nav_config = LaunchConfiguration('body_relative_nav_config')
    detector_config = LaunchConfiguration('detector_config')
    pose_topic = LaunchConfiguration('pose_topic')
    pose_topic_type = LaunchConfiguration('pose_topic_type')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    serial_device = LaunchConfiguration('serial_device')
    nav_backend = LaunchConfiguration('nav_backend')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_detector = LaunchConfiguration('enable_detector')
    enable_orange_pole_relative_nav = LaunchConfiguration('enable_orange_pole_relative_nav')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    enable_point_lio = LaunchConfiguration('enable_point_lio')
    point_lio_enable_nav2 = LaunchConfiguration('point_lio_enable_nav2')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'obstacle_race_system.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'body_relative_nav_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'orange_pole_body_relative_test_v4.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'detector_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'orange_pole_detector.yaml',
            ]),
        ),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('pose_topic_type', default_value='odometry'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('nav_backend', default_value='pose_controller'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_detector', default_value='true'),
        DeclareLaunchArgument('enable_orange_pole_relative_nav', default_value='true'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='true'),
        DeclareLaunchArgument('enable_point_lio', default_value='false'),
        DeclareLaunchArgument('point_lio_enable_nav2', default_value='false'),
        DeclareLaunchArgument('color_profile', default_value='640,480,15'),
        DeclareLaunchArgument('depth_profile', default_value='640,480,15'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('point_lio'),
                    'launch',
                    'mapping_mid360.launch.py',
                ])
            ),
            condition=IfCondition(enable_point_lio),
            launch_arguments={
                'rviz': 'false',
                'enable_nav2_bringup': point_lio_enable_nav2,
            }.items(),
        ),

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
            condition=IfCondition(enable_detector),
            parameters=[detector_config],
        ),

        Node(
            package='auto_nav_pkg',
            executable='orange_pole_relative_nav_node',
            name='orange_pole_relative_nav',
            output='screen',
            condition=IfCondition(enable_orange_pole_relative_nav),
            parameters=[
                system_config,
                body_relative_nav_config,
                {
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='nav_executor_node',
            name='nav_executor',
            output='screen',
            parameters=[
                system_config,
                {
                    'backend': nav_backend,
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'cmd_vel_topic': cmd_vel_topic,
                },
            ],
        ),

        Node(
            package='quadruped_step_mapper',
            executable='cmd_vel_to_serial_node',
            name='cmd_vel_to_serial',
            output='screen',
            condition=IfCondition(enable_serial_bridge),
            parameters=[
                PathJoinSubstitution([step_share, 'config', 'cmd_vel_to_serial.yaml']),
                {
                    'input_cmd_vel_topic': cmd_vel_topic,
                    'serial_device': serial_device,
                    'mode_topic': '/dog_mode_current',
                    'odom_y_correction_enabled': False,
                },
            ],
        ),
    ])
