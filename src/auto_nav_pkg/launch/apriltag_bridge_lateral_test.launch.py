from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')
    apriltag_share = FindPackageShare('apriltag_ros')
    step_share = FindPackageShare('quadruped_step_mapper')

    config = LaunchConfiguration('config')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_apriltag = LaunchConfiguration('enable_apriltag')
    enable_control = LaunchConfiguration('enable_control')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    serial_device = LaunchConfiguration('serial_device')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    target_tag_x_m = LaunchConfiguration('target_tag_x_m')
    robot_center_offset_x_m = LaunchConfiguration('robot_center_offset_x_m')
    bridge_lateral_control_sign = LaunchConfiguration('bridge_lateral_control_sign')
    bridge_lateral_forward_speed_mps = LaunchConfiguration(
        'bridge_lateral_forward_speed_mps'
    )
    bridge_lateral_deadband_m = LaunchConfiguration('bridge_lateral_deadband_m')
    bridge_lateral_max_abs_error_m = LaunchConfiguration('bridge_lateral_max_abs_error_m')
    bridge_lateral_max_abs_angular_z = LaunchConfiguration(
        'bridge_lateral_max_abs_angular_z'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'apriltag_bridge_lateral_test.yaml',
            ]),
        ),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_apriltag', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='true'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='false'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('target_tag_x_m', default_value='-0.433'),
        DeclareLaunchArgument('robot_center_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('bridge_lateral_control_sign', default_value='-1.0'),
        DeclareLaunchArgument('bridge_lateral_forward_speed_mps', default_value='0.05'),
        DeclareLaunchArgument('bridge_lateral_deadband_m', default_value='0.03'),
        DeclareLaunchArgument('bridge_lateral_max_abs_error_m', default_value='0.45'),
        DeclareLaunchArgument('bridge_lateral_max_abs_angular_z', default_value='0.16'),

        GroupAction(
            scoped=True,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            auto_share,
                            'launch',
                            'apriltag_align_test.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'config': config,
                        'apriltag_config': PathJoinSubstitution([
                            apriltag_share,
                            'cfg',
                            'tags_36h11.yaml',
                        ]),
                        'target_frame_id': 'camera_color_optical_frame',
                        'target_child_frame_id': 'bridge',
                        'target_tag_id': '15',
                        'target_tag_x_m': target_tag_x_m,
                        'robot_center_offset_x_m': robot_center_offset_x_m,
                        'alignment_mode': 'center_x_offset',
                        'enable_camera': enable_camera,
                        'enable_apriltag': enable_apriltag,
                        'enable_control': enable_control,
                        'enable_serial_bridge': 'false',
                        'control_output_mode': 'hybrid',
                        'serial_device': serial_device,
                        'cmd_vel_topic': cmd_vel_topic,
                        'pose_topic': pose_topic,
                        'bridge_lateral_calibration_enabled': 'true',
                        'bridge_lateral_control_sign': bridge_lateral_control_sign,
                        'bridge_lateral_forward_speed_mps': bridge_lateral_forward_speed_mps,
                        'bridge_lateral_deadband_m': bridge_lateral_deadband_m,
                        'bridge_lateral_max_abs_error_m': bridge_lateral_max_abs_error_m,
                        'bridge_lateral_max_abs_angular_z': bridge_lateral_max_abs_angular_z,
                        'enable_forward_approach': 'false',
                        'lateral_shift_sequence_enabled': 'false',
                        'limit_bar_duck_sequence_enabled': 'false',
                        'stale_timeout_sec': '0.60',
                        'pose_stale_timeout_sec': '0.80',
                    }.items(),
                ),
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
                    'step_override_topic': '/serial_step_override',
                    'step_override_timeout_sec': 0.80,
                    'odom_y_correction_enabled': False,
                    'path_lateral_correction_enabled': False,
                    'path_yaw_correction_enabled': False,
                    'crouch_yaw_correction_enabled': False,
                    'step_override_small_opposite_sign_shape_enabled': False,
                    'cmd_vel_small_opposite_sign_shape_enabled': False,
                    'min_output_step_norm_modes': [-999],
                    'min_output_step_norm_override_modes': [-999],
                    'min_output_step_norm_override_norms': [0.0],
                    'uphill_segment_step_floor_enabled': False,
                    'uphill_pitch_step_floor_enabled': False,
                },
            ],
        ),
    ])
