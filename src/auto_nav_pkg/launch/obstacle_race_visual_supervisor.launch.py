from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')
    step_share = FindPackageShare('quadruped_step_mapper')

    race_mode = LaunchConfiguration('race_mode')
    enable_race_mode_selection = LaunchConfiguration(
        'enable_race_mode_selection'
    )
    system_config = LaunchConfiguration('system_config')
    waypoints_file = LaunchConfiguration('waypoints_file')
    body_relative_nav_config = LaunchConfiguration('body_relative_nav_config')
    stable_trigger_config = LaunchConfiguration('stable_trigger_config')
    detector_config = LaunchConfiguration('detector_config')
    limit_bar_config = LaunchConfiguration('limit_bar_config')
    hurdle_config = LaunchConfiguration('hurdle_config')
    slope_detector_config = LaunchConfiguration('slope_detector_config')
    upstairs_detector_config = LaunchConfiguration('upstairs_detector_config')
    supervisor_config = LaunchConfiguration('supervisor_config')
    pose_topic = LaunchConfiguration('pose_topic')
    pose_topic_type = LaunchConfiguration('pose_topic_type')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    serial_device = LaunchConfiguration('serial_device')
    supervisor_wait_for_start_signal = LaunchConfiguration(
        'supervisor_wait_for_start_signal'
    )
    nav_backend = LaunchConfiguration('nav_backend')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_apriltag = LaunchConfiguration('enable_apriltag')
    enable_depth = LaunchConfiguration('enable_depth')
    enable_align_depth = LaunchConfiguration('enable_align_depth')
    enable_sync = LaunchConfiguration('enable_sync')
    enable_aux_detectors = LaunchConfiguration('enable_aux_detectors')
    enable_slope_detector = LaunchConfiguration('enable_slope_detector')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    enable_point_lio = LaunchConfiguration('enable_point_lio')
    point_lio_enable_nav2 = LaunchConfiguration('point_lio_enable_nav2')
    apriltag_config = LaunchConfiguration('apriltag_config')
    supervisor_overlay_config = LaunchConfiguration('supervisor_overlay_config')
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'race_mode',
            default_value='1',
            description='Route mode: 0=standard waypoints/v4 pole config, 1=mirror waypoints/v1 pole config.',
        ),
        DeclareLaunchArgument(
            'enable_race_mode_selection',
            default_value='true',
            description=(
                'Enable mode-dependent route files. Set false to force the '
                'original standard waypoints and v4 pole configuration.'
            ),
        ),
        DeclareLaunchArgument(
            'system_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'obstacle_race_system.yaml']),
        ),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                PythonExpression([
                    "'obstacle_waypoints_mirror.yaml' if '",
                    enable_race_mode_selection,
                    "'.lower() in ('true', '1', 'yes', 'on') and '",
                    race_mode,
                    "' == '1' else 'obstacle_waypoints.yaml'",
                ]),
            ]),
        ),
        #绕杆的参数
        DeclareLaunchArgument(
            'body_relative_nav_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                PythonExpression([
                    "'orange_pole_body_relative_test_v1.yaml' if '",
                    enable_race_mode_selection,
                    "'.lower() in ('true', '1', 'yes', 'on') and '",
                    race_mode,
                    "' == '1' else 'orange_pole_body_relative_test_v4.yaml'",
                ]),
            ]),
        ),
        DeclareLaunchArgument(
            'stable_trigger_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'orange_pole_stable_trigger.yaml']),
        ),
        DeclareLaunchArgument(
            'detector_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'orange_pole_detector.yaml']),
        ),
        DeclareLaunchArgument(
            'limit_bar_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'limit_bar_duck_test.yaml']),
        ),
        DeclareLaunchArgument(
            'hurdle_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'orange_hurdle_jump_test.yaml']),
        ),
        DeclareLaunchArgument(
            'slope_detector_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'slope_branch_detector.yaml']),
        ),
        DeclareLaunchArgument(
            'upstairs_detector_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'upstairs_detector.yaml']),
        ),
        DeclareLaunchArgument(
            'supervisor_config',
            default_value=PathJoinSubstitution([auto_share, 'config', 'obstacle_race_supervisor.yaml']),
        ),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('pose_topic_type', default_value='odometry'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument(
            'supervisor_wait_for_start_signal',
            default_value='true',
            description=(
                'Wait for a serial start signal inside the supervisor. '
                'The external serial mode launcher sets this false because it consumed the first start frame.'
            ),
        ),
        DeclareLaunchArgument('nav_backend', default_value='pose_controller'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        # 2026-05-17: AprilTag is used by wall_jump_prep to end crouch-walk at
        # camera_color_optical_frame->base z <= 0.45m. Rollback: launch with
        # enable_apriltag:=false or set wall_jump_prep.apriltag_arrival_enabled=false.
        DeclareLaunchArgument('enable_apriltag', default_value='true'),
        DeclareLaunchArgument('enable_depth', default_value='true'),
        DeclareLaunchArgument('enable_align_depth', default_value='true'),
        DeclareLaunchArgument('enable_sync', default_value='true'),
        DeclareLaunchArgument('enable_aux_detectors', default_value='true'),
        # 2026-06-15: supervisor 默认改为 roll 判斜坡；只有显式打开这个开关时才启动
        # slope_branch_detector。
        DeclareLaunchArgument('enable_slope_detector', default_value='false'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='true'),
        DeclareLaunchArgument('enable_point_lio', default_value='false'),
        DeclareLaunchArgument('point_lio_enable_nav2', default_value='false'),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('apriltag_ros'),
                'cfg',
                'tags_36h11.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'supervisor_overlay_config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'obstacle_race_supervisor_overlay.yaml',
            ]),
        ),
        # 2026-05-24
        # Purpose: reduce RealSense USB/UVC pressure after field logs showed
        # color frames dropping to ~8-10Hz and dmesg reporting UVC -71 errors.
        # Behavior: default color/depth stream profiles use 424x240@15 for a
        # steadier full-vision run with AprilTag plus HSV/depth detectors.
        # Rollback: change these defaults back to 640,480,30 after the USB link
        # is stable under full visual load.
        DeclareLaunchArgument('color_profile', default_value='424,240,15'),
        DeclareLaunchArgument('depth_profile', default_value='424,240,15'),

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
                'enable_depth': enable_depth,
                'enable_sync': enable_sync,
                'align_depth.enable': enable_align_depth,
                # 2026-05-22: AprilTag 和几路 HSV 节点都以 sensor_data QoS
                # 订阅图像/相机内参。将 D435 color/depth 图像与 camera_info
                # 全部切到 SENSOR_DATA，减少默认可靠 QoS 累积旧帧导致的
                # image/camera_info 配对滞后。
                # Rollback: 删除下面四项，恢复 realsense 默认 image/system_default
                # 与 camera_info/default。
                'color_qos': 'SENSOR_DATA',
                'color_info_qos': 'SENSOR_DATA',
                'depth_qos': 'SENSOR_DATA',
                'depth_info_qos': 'SENSOR_DATA',
                'decimation_filter.enable': 'false',
                'pointcloud.enable': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                'rgb_camera.profile': color_profile,
                'depth_module.profile': depth_profile,
            }.items(),
        ),

        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag',
            namespace='apriltag',
            output='screen',
            condition=IfCondition(enable_apriltag),
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
            parameters=[apriltag_config],
        ),

        Node(
            package='auto_nav_pkg',
            executable='orange_pole_detector_node',
            name='orange_pole_detector',
            output='screen',
            condition=IfCondition(enable_aux_detectors),
            parameters=[detector_config],
        ),

        Node(
            package='auto_nav_pkg',
            executable='limit_bar_duck_test_node',
            name='limit_bar_duck_test',
            output='screen',
            condition=IfCondition(enable_aux_detectors),
            parameters=[
                limit_bar_config,
                {
                    'detector_only': True,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='orange_hurdle_jump_test_node',
            name='orange_hurdle_jump_test',
            output='screen',
            condition=IfCondition(enable_aux_detectors),
            parameters=[
                hurdle_config,
                {
                    'detector_only': True,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='slope_branch_detector_node',
            name='slope_branch_detector',
            output='screen',
            condition=IfCondition(
                PythonExpression([
                    "'",
                    enable_aux_detectors,
                    "' == 'true' and '",
                    enable_slope_detector,
                    "' == 'true'",
                ])
            ),
            parameters=[slope_detector_config],
        ),

        Node(
            package='auto_nav_pkg',
            executable='upstairs_detector_node',
            name='upstairs_detector',
            output='screen',
            condition=IfCondition(enable_aux_detectors),
            parameters=[upstairs_detector_config],
        ),

        Node(
            package='auto_nav_pkg',
            # Production full-flow pole branch. The supervisor activates this
            # node only in POLE_ACTIVE and consumes its FINISHED state to
            # return to SEARCH. Set lidar_slalom_enabled=false in the selected
            # V1/V4 YAML to roll back without changing those interfaces.
            executable='orange_pole_lidar_slalom_node',
            name='orange_pole_relative_nav',
            output='screen',
            parameters=[
                system_config,
                body_relative_nav_config,
                stable_trigger_config,
                {
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'active': False,
                    'active_topic': '/orange_pole_relative_nav/active',
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='obstacle_manager_node',
            name='stairs_manager',
            output='screen',
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
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'active': False,
                    'active_topic': '/obstacle_supervisor/stairs_active',
                    'start_waypoint_index': 0,
                    'stop_after_waypoint_index': 0,
                    'step_override_topic': '/serial_step_override',
                    'path_reference_start_topic': '/serial_path_reference_start',
                    'path_reference_topic': '/serial_path_reference',
                    'uphill_step_floor_state_topic': '/race_manager/uphill_step_floor_enabled',
                    'path_correction_profile_topic': '/race_manager/path_correction_profile',
                    'allow_frame_mismatch': False,
                    'default_frame_id': 'camera_init',
                    'arrival_tolerance': 0.20,
                    'use_yaw_tolerance': False,
                    'yaw_tolerance_deg': 20.0,
                    'allow_goal_pass_through': True,
                    'pass_through_lateral_tolerance': 0.25,
                    'goal_timeout_sec': 45.0,
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
                    'latch_mode_until_next_waypoint': True,
                    'mode_transition_delay_sec': 0.15,
                },
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='obstacle_manager_node',
            name='slope_manager',
            output='screen',
            parameters=[
                system_config,
                {
                    'waypoints_file': waypoints_file,
                    'goal_topic': '/goal_pose',
                    'mode_topic': '/dog_mode_current',
                    'arrival_topic': '/slope_manager/arrival_status',
                    'state_topic': '/slope_manager/state',
                    'feedback_topic': '/slope_manager/feedback_log',
                    'current_waypoint_topic': '/slope_manager/current_waypoint',
                    'monitor_state_topic': '/slope_manager/monitor_state',
                    'monitor_feedback_topic': '/slope_manager/monitor_feedback',
                    'executor_state_topic': '/nav_executor/state',
                    'pose_topic': pose_topic,
                    'pose_topic_type': pose_topic_type,
                    'active': False,
                    'active_topic': '/obstacle_supervisor/slope_active',
                    'restart_from_waypoint_topic': '/slope_manager/restart_from_waypoint',
                    'start_waypoint_index': 1,
                    'stop_after_waypoint_index': -1,
                    'inherit_previous_waypoint_segment_state': True,
                    'inherited_segment_mode_override': 7,
                    'step_override_topic': '/serial_step_override',
                    'path_reference_start_topic': '/serial_path_reference_start',
                    'path_reference_topic': '/serial_path_reference',
                    'startup_reference_topic': '/slope_align_reference',
                    'startup_reference_timeout_sec': 15.0,
                    'runtime_route_yaw_reference_enabled': True,
                    'route_yaw_reference_topic': '/slope_route_yaw_reference',
                    'route_yaw_reference_timeout_sec': 0.0,
                    'bridge_retry_yaw_reference_topic': '/slope_bridge_retry_yaw_reference',
                    'bridge_retry_yaw_reference_timeout_sec': 0.0,
                    'uphill_step_floor_state_topic': '/race_manager/uphill_step_floor_enabled',
                    'path_correction_profile_topic': '/race_manager/path_correction_profile',
                    # ID26/base uses a dedicated latest-only TF path so shared
                    # /tf backlog cannot delay the wall_jump_prep distance gate.
                    # Other AprilTag frames continue to use the original /tf subscription.
                    'apriltag_base_tf_topic': '/apriltag/tag_tf',
                    'apriltag_base_tf_queue_depth': 1,
                    'apriltag_base_tf_best_effort': True,
                    'apriltag_base_tf_frame_id': 'camera_color_optical_frame',
                    'apriltag_base_tf_child_frame_id': 'base',
                    'goal_timeout_sec': 90.0,
                    'pose_stale_timeout_sec': 5.0,
                    'idle_mode': 1,
                    'finish_mode': 0,
                    'startup_delay_sec': 0.0,
                    'settle_after_arrival_sec': 0.20,
                    'retry_delay_sec': 1.00,
                    'max_goal_retries': 2,
                    'skip_waypoint_on_failure': False,
                    'latch_mode_until_next_waypoint': True,
                    'mode_transition_delay_sec': 0.15,
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
            respawn=True,
            respawn_delay=1.0,
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

        Node(
            package='auto_nav_pkg',
            executable='obstacle_race_supervisor_node',
            name='obstacle_race_supervisor',
            output='screen',
            parameters=[
                supervisor_config,
                supervisor_overlay_config,
                {
                    'pose_topic': pose_topic,
                    'sandpit_bypass_file': waypoints_file,
                    'pole_bypass_file': body_relative_nav_config,
                    'wait_for_start_signal': ParameterValue(
                        supervisor_wait_for_start_signal,
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
