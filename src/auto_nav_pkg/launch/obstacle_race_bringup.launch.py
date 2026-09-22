from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    auto_share = get_package_share_directory('auto_nav_pkg')
    step_share = get_package_share_directory('quadruped_step_mapper')

    system_config = LaunchConfiguration('system_config')
    pose_topic = LaunchConfiguration('pose_topic')
    pose_topic_type = LaunchConfiguration('pose_topic_type')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    serial_device = LaunchConfiguration('serial_device')
    nav_backend = LaunchConfiguration('nav_backend')
    waypoints_file = LaunchConfiguration('waypoints_file')
    arrival_tolerance = LaunchConfiguration('arrival_tolerance')
    goal_timeout_sec = LaunchConfiguration('goal_timeout_sec')
    use_yaw_tolerance = LaunchConfiguration('use_yaw_tolerance')
    yaw_tolerance_deg = LaunchConfiguration('yaw_tolerance_deg')
    allow_frame_mismatch = LaunchConfiguration('allow_frame_mismatch')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    enable_obstacle_manager = LaunchConfiguration('enable_obstacle_manager')
    enable_stairs_manager = LaunchConfiguration('enable_stairs_manager')
    enable_yolo_relative_nav = LaunchConfiguration('enable_yolo_relative_nav')
    enable_apriltag = LaunchConfiguration('enable_apriltag')
    latch_mode_until_next_waypoint = LaunchConfiguration('latch_mode_until_next_waypoint')
    enable_nav2_bringup = LaunchConfiguration('enable_nav2_bringup')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    nav2_autostart = LaunchConfiguration('nav2_autostart')
    nav2_log_level = LaunchConfiguration('nav2_log_level')
    nav2_bringup_condition = IfCondition(PythonExpression([
        "'", enable_nav2_bringup, "' == 'true' and '", nav_backend, "' == 'nav2'"
    ]))

    return LaunchDescription([
        DeclareLaunchArgument(
            'system_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'obstacle_race_system.yaml']),
        ),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('pose_topic_type', default_value='odometry'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('nav_backend', default_value='pose_controller'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([auto_share, 'config', 'obstacle_waypoints.yaml']),
        ),
        DeclareLaunchArgument('arrival_tolerance', default_value='0.20'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('use_yaw_tolerance', default_value='false'),
        DeclareLaunchArgument('yaw_tolerance_deg', default_value='20.0'),
        DeclareLaunchArgument('allow_frame_mismatch', default_value='false'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='true'),
        DeclareLaunchArgument('enable_obstacle_manager', default_value='true'),
        DeclareLaunchArgument('enable_stairs_manager', default_value='true'),
        DeclareLaunchArgument('enable_yolo_relative_nav', default_value='false'),
        # 2026-05-17: AprilTag provides /tf distance for wall_jump_prep stop.
        # Rollback: enable_apriltag:=false or disable apriltag_arrival_enabled in the waypoint.
        DeclareLaunchArgument('enable_apriltag', default_value='true'),
        DeclareLaunchArgument('latch_mode_until_next_waypoint', default_value='true'),
        # 2026-05-02: Nav2 is normally started by point_lio_ros2/mapping_mid360.launch.py
        # so this terminal stays focused on obstacle_manager/nav_executor/serial logs.
        # Keep this fallback switch for rollback: enable_nav2_bringup:=true nav_backend:=nav2.
        DeclareLaunchArgument('enable_nav2_bringup', default_value='false'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([auto_share, 'config', 'nav2_pointlio_navigation.yaml']),
        ),
        DeclareLaunchArgument('nav2_autostart', default_value='true'),
        DeclareLaunchArgument('nav2_log_level', default_value='info'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_bringup'),
                    'launch',
                    'navigation_launch.py',
                ])
            ),
            condition=nav2_bringup_condition,
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': nav2_autostart,
                'params_file': nav2_params_file,
                'use_composition': 'False',
                'use_respawn': 'False',
                'log_level': nav2_log_level,
            }.items(),
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('apriltag_ros'),
                    'launch',
                    'camera_36h11.launch.yml',
                ])
            ),
            condition=IfCondition(enable_apriltag),
        ),

        Node(
            package='auto_nav_pkg',
            executable='obstacle_manager_node',
            name='obstacle_manager',
            output='screen',
            condition=IfCondition(enable_obstacle_manager),
            parameters=[
                system_config,
                {
                    'waypoints_file': waypoints_file,
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'arrival_tolerance': arrival_tolerance,
                    'goal_timeout_sec': goal_timeout_sec,
                    'use_yaw_tolerance': use_yaw_tolerance,
                    'yaw_tolerance_deg': yaw_tolerance_deg,
                    'allow_frame_mismatch': allow_frame_mismatch,
                    'latch_mode_until_next_waypoint': latch_mode_until_next_waypoint,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='obstacle_manager_node',
            name='stairs_manager',
            output='screen',
            condition=IfCondition(enable_stairs_manager),
            parameters=[
                system_config,
                {
                    'waypoints_file': waypoints_file,
                    'goal_topic': '/goal_pose',
                    'mode_topic': '/dog_mode_current',
                    'arrival_topic': '/stairs_manager/arrival_status',
                    'state_topic': '/stairs_manager/state',
                    'feedback_topic': '/stairs_manager/feedback_log',
                    'current_waypoint_topic': '/stairs_manager/current_waypoint',
                    'monitor_state_topic': '/stairs_manager/monitor_state',
                    'monitor_feedback_topic': '/stairs_manager/monitor_feedback',
                    'executor_state_topic': '/nav_executor/state',
                    'active': False,
                    'active_topic': '/obstacle_supervisor/stairs_active',
                    'start_waypoint_index': 0,
                    'stop_after_waypoint_index': 0,
                    'step_override_topic': '/serial_step_override',
                    'path_reference_start_topic': '/serial_path_reference_start',
                    'path_reference_topic': '/serial_path_reference',
                    'uphill_step_floor_state_topic': '/race_manager/uphill_step_floor_enabled',
                    'path_correction_profile_topic': '/race_manager/path_correction_profile',
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'allow_frame_mismatch': allow_frame_mismatch,
                    'default_frame_id': 'camera_init',
                    'arrival_tolerance': arrival_tolerance,
                    'use_yaw_tolerance': use_yaw_tolerance,
                    'yaw_tolerance_deg': yaw_tolerance_deg,
                    'allow_goal_pass_through': True,
                    'pass_through_lateral_tolerance': 0.25,
                    'goal_timeout_sec': goal_timeout_sec,
                    'pose_stale_timeout_sec': 5.0,
                    'monitor_rate_hz': 10.0,
                    'progress_log_interval_sec': 1.0,
                    'arrival_check_on_pose_update': True,
                    'use_min_dist_for_yaw_waypoint': True,
                    'idle_mode': 1,
                    'finish_mode': 0,
                    'startup_delay_sec': 0.0,
                    'settle_after_arrival_sec': 0.20,
                    'retry_delay_sec': 1.00,
                    'max_goal_retries': 2,
                    'skip_waypoint_on_failure': False,
                    'latch_mode_until_next_waypoint': latch_mode_until_next_waypoint,
                    'mode_transition_delay_sec': 0.15,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='yolo_relative_nav_node',
            name='yolo_relative_nav',
            output='screen',
            condition=IfCondition(enable_yolo_relative_nav),
            parameters=[
                system_config,
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
