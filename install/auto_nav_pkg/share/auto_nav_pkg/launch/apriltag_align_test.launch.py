from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')
    apriltag_share = FindPackageShare('apriltag_ros')
    step_share = FindPackageShare('quadruped_step_mapper')

    config = LaunchConfiguration('config')
    apriltag_config = LaunchConfiguration('apriltag_config')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    serial_device = LaunchConfiguration('serial_device')
    enable_camera = LaunchConfiguration('enable_camera')
    enable_apriltag = LaunchConfiguration('enable_apriltag')
    enable_control = LaunchConfiguration('enable_control')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    control_output_mode = LaunchConfiguration('control_output_mode')
    target_frame_id = LaunchConfiguration('target_frame_id')
    target_child_frame_id = LaunchConfiguration('target_child_frame_id')
    target_tag_id = LaunchConfiguration('target_tag_id')
    target_offset_x_m = LaunchConfiguration('target_offset_x_m')
    target_tag_x_m = LaunchConfiguration('target_tag_x_m')
    robot_center_offset_x_m = LaunchConfiguration('robot_center_offset_x_m')
    alignment_mode = LaunchConfiguration('alignment_mode')
    center_control_sign = LaunchConfiguration('center_control_sign')
    pose_control_sign = LaunchConfiguration('pose_control_sign')
    max_abs_angular_z = LaunchConfiguration('max_abs_angular_z')
    yaw_deadband_deg = LaunchConfiguration('yaw_deadband_deg')
    enable_forward_approach = LaunchConfiguration('enable_forward_approach')
    target_distance_m = LaunchConfiguration('target_distance_m')
    max_forward_speed_mps = LaunchConfiguration('max_forward_speed_mps')
    forward_max_yaw_error_deg = LaunchConfiguration('forward_max_yaw_error_deg')
    approach_requires_aligned = LaunchConfiguration('approach_requires_aligned')
    bridge_lateral_calibration_enabled = LaunchConfiguration(
        'bridge_lateral_calibration_enabled'
    )
    bridge_lateral_deadband_m = LaunchConfiguration('bridge_lateral_deadband_m')
    bridge_lateral_hold_sec = LaunchConfiguration('bridge_lateral_hold_sec')
    bridge_lateral_forward_speed_mps = LaunchConfiguration(
        'bridge_lateral_forward_speed_mps'
    )
    bridge_lateral_kp_radps_per_m = LaunchConfiguration('bridge_lateral_kp_radps_per_m')
    bridge_lateral_min_abs_angular_z = LaunchConfiguration(
        'bridge_lateral_min_abs_angular_z'
    )
    bridge_lateral_max_abs_angular_z = LaunchConfiguration(
        'bridge_lateral_max_abs_angular_z'
    )
    bridge_lateral_control_sign = LaunchConfiguration('bridge_lateral_control_sign')
    bridge_lateral_max_abs_error_m = LaunchConfiguration('bridge_lateral_max_abs_error_m')
    bridge_lateral_finish_after_hold = LaunchConfiguration(
        'bridge_lateral_finish_after_hold'
    )
    stale_timeout_sec = LaunchConfiguration('stale_timeout_sec')
    pose_stale_timeout_sec = LaunchConfiguration('pose_stale_timeout_sec')
    lock_reference_on_tag_loss = LaunchConfiguration('lock_reference_on_tag_loss')
    locked_reference_along_sign = LaunchConfiguration('locked_reference_along_sign')
    lock_reference_update_while_visible = LaunchConfiguration(
        'lock_reference_update_while_visible'
    )
    lateral_shift_sequence_enabled = LaunchConfiguration('lateral_shift_sequence_enabled')
    lateral_shift_pre_align_enabled = LaunchConfiguration('lateral_shift_pre_align_enabled')
    lateral_shift_pre_align_mode = LaunchConfiguration('lateral_shift_pre_align_mode')
    lateral_shift_pre_align_use_visual_feedback = LaunchConfiguration(
        'lateral_shift_pre_align_use_visual_feedback'
    )
    lateral_shift_continue_after_tag_loss = LaunchConfiguration(
        'lateral_shift_continue_after_tag_loss'
    )
    lateral_shift_pre_align_timeout_sec = LaunchConfiguration(
        'lateral_shift_pre_align_timeout_sec'
    )
    lateral_shift_pre_align_max_visual_error_deg = LaunchConfiguration(
        'lateral_shift_pre_align_max_visual_error_deg'
    )
    lateral_shift_pre_align_hold_sec = LaunchConfiguration('lateral_shift_pre_align_hold_sec')
    lateral_shift_pre_align_tolerance_deg = LaunchConfiguration(
        'lateral_shift_pre_align_tolerance_deg'
    )
    lateral_shift_yaw_deg = LaunchConfiguration('lateral_shift_yaw_deg')
    lateral_shift_yaw_sign = LaunchConfiguration('lateral_shift_yaw_sign')
    lateral_shift_deadband_m = LaunchConfiguration('lateral_shift_deadband_m')
    lateral_shift_distance_scale = LaunchConfiguration('lateral_shift_distance_scale')
    lateral_shift_max_distance_m = LaunchConfiguration('lateral_shift_max_distance_m')
    lateral_shift_use_euclidean_progress = LaunchConfiguration(
        'lateral_shift_use_euclidean_progress'
    )
    lateral_shift_final_yaw_gate_deg = LaunchConfiguration('lateral_shift_final_yaw_gate_deg')
    lateral_shift_duck_entry_yaw_gate_deg = LaunchConfiguration(
        'lateral_shift_duck_entry_yaw_gate_deg'
    )
    lateral_shift_duck_entry_hold_sec = LaunchConfiguration(
        'lateral_shift_duck_entry_hold_sec'
    )
    limit_bar_duck_sequence_enabled = LaunchConfiguration('limit_bar_duck_sequence_enabled')
    duck_prepare_mode = LaunchConfiguration('duck_prepare_mode')
    duck_prepare_left_norm = LaunchConfiguration('duck_prepare_left_norm')
    duck_prepare_right_norm = LaunchConfiguration('duck_prepare_right_norm')
    duck_prepare_delay_sec = LaunchConfiguration('duck_prepare_delay_sec')
    duck_walk_mode = LaunchConfiguration('duck_walk_mode')
    duck_walk_left_norm = LaunchConfiguration('duck_walk_left_norm')
    duck_walk_right_norm = LaunchConfiguration('duck_walk_right_norm')
    duck_walk_yaw_correction_enabled = LaunchConfiguration('duck_walk_yaw_correction_enabled')
    duck_walk_yaw_gain_norm_per_rad = LaunchConfiguration('duck_walk_yaw_gain_norm_per_rad')
    duck_walk_yaw_deadband_deg = LaunchConfiguration('duck_walk_yaw_deadband_deg')
    duck_walk_yaw_max_delta_norm = LaunchConfiguration('duck_walk_yaw_max_delta_norm')
    duck_walk_distance_m = LaunchConfiguration('duck_walk_distance_m')
    duck_walk_timeout_sec = LaunchConfiguration('duck_walk_timeout_sec')
    duck_resume_mode = LaunchConfiguration('duck_resume_mode')
    duck_resume_left_norm = LaunchConfiguration('duck_resume_left_norm')
    duck_resume_right_norm = LaunchConfiguration('duck_resume_right_norm')
    duck_resume_delay_sec = LaunchConfiguration('duck_resume_delay_sec')
    near_approach_use_cmd_vel = LaunchConfiguration('near_approach_use_cmd_vel')
    near_approach_distance_error_m = LaunchConfiguration('near_approach_distance_error_m')
    step_override_forward_left_norm = LaunchConfiguration('step_override_forward_left_norm')
    step_override_forward_right_norm = LaunchConfiguration('step_override_forward_right_norm')
    step_override_forward_scale_enabled = LaunchConfiguration(
        'step_override_forward_scale_enabled'
    )
    step_override_forward_min_scale = LaunchConfiguration('step_override_forward_min_scale')
    step_override_forward_yaw_correction_enabled = LaunchConfiguration(
        'step_override_forward_yaw_correction_enabled'
    )
    step_override_forward_yaw_gain_norm_per_radps = LaunchConfiguration(
        'step_override_forward_yaw_gain_norm_per_radps'
    )
    step_override_forward_yaw_max_delta_norm = LaunchConfiguration(
        'step_override_forward_yaw_max_delta_norm'
    )
    step_override_turn_cw_left_norm = LaunchConfiguration('step_override_turn_cw_left_norm')
    step_override_turn_cw_right_norm = LaunchConfiguration('step_override_turn_cw_right_norm')
    step_override_turn_ccw_left_norm = LaunchConfiguration('step_override_turn_ccw_left_norm')
    step_override_turn_ccw_right_norm = LaunchConfiguration('step_override_turn_ccw_right_norm')
    step_override_turn_scale_enabled = LaunchConfiguration('step_override_turn_scale_enabled')
    step_override_turn_min_scale = LaunchConfiguration('step_override_turn_min_scale')
    step_override_duck_align_turn_enabled = LaunchConfiguration(
        'step_override_duck_align_turn_enabled'
    )
    step_override_duck_align_cw_left_norm = LaunchConfiguration(
        'step_override_duck_align_cw_left_norm'
    )
    step_override_duck_align_cw_right_norm = LaunchConfiguration(
        'step_override_duck_align_cw_right_norm'
    )
    step_override_duck_align_ccw_left_norm = LaunchConfiguration(
        'step_override_duck_align_ccw_left_norm'
    )
    step_override_duck_align_ccw_right_norm = LaunchConfiguration(
        'step_override_duck_align_ccw_right_norm'
    )
    color_profile = LaunchConfiguration('color_profile')
    depth_profile = LaunchConfiguration('depth_profile')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=PathJoinSubstitution([
                auto_share,
                'config',
                'apriltag_align_test.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'apriltag_config',
            default_value=PathJoinSubstitution([
                apriltag_share,
                'cfg',
                'tags_36h11.yaml',
            ]),
        ),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_apriltag', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='false'),
        DeclareLaunchArgument('control_output_mode', default_value='hybrid'),
        DeclareLaunchArgument('target_frame_id', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('target_child_frame_id', default_value='object'),
        DeclareLaunchArgument('target_tag_id', default_value='14'),
        DeclareLaunchArgument('target_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('target_tag_x_m', default_value='nan'),
        DeclareLaunchArgument('robot_center_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('alignment_mode', default_value='center_x'),
        DeclareLaunchArgument('center_control_sign', default_value='-1.0'),
        DeclareLaunchArgument('pose_control_sign', default_value='1.0'),
        DeclareLaunchArgument('max_abs_angular_z', default_value='0.25'),
        DeclareLaunchArgument('yaw_deadband_deg', default_value='1.5'),
        DeclareLaunchArgument('enable_forward_approach', default_value='false'),
        DeclareLaunchArgument('target_distance_m', default_value='0.45'),
        DeclareLaunchArgument('max_forward_speed_mps', default_value='0.12'),
        DeclareLaunchArgument('forward_max_yaw_error_deg', default_value='3.0'),
        DeclareLaunchArgument('approach_requires_aligned', default_value='false'),
        DeclareLaunchArgument('bridge_lateral_calibration_enabled', default_value='false'),
        DeclareLaunchArgument('bridge_lateral_deadband_m', default_value='0.03'),
        DeclareLaunchArgument('bridge_lateral_hold_sec', default_value='0.25'),
        DeclareLaunchArgument('bridge_lateral_forward_speed_mps', default_value='0.05'),
        DeclareLaunchArgument('bridge_lateral_kp_radps_per_m', default_value='1.5'),
        DeclareLaunchArgument('bridge_lateral_min_abs_angular_z', default_value='0.02'),
        DeclareLaunchArgument('bridge_lateral_max_abs_angular_z', default_value='0.18'),
        DeclareLaunchArgument('bridge_lateral_control_sign', default_value='-1.0'),
        DeclareLaunchArgument('bridge_lateral_max_abs_error_m', default_value='0.45'),
        DeclareLaunchArgument('bridge_lateral_finish_after_hold', default_value='true'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.30'),
        DeclareLaunchArgument('pose_stale_timeout_sec', default_value='0.80'),
        DeclareLaunchArgument('lock_reference_on_tag_loss', default_value='false'),
        DeclareLaunchArgument('locked_reference_along_sign', default_value='1.0'),
        DeclareLaunchArgument('lock_reference_update_while_visible', default_value='true'),
        DeclareLaunchArgument('lateral_shift_sequence_enabled', default_value='false'),
        DeclareLaunchArgument('lateral_shift_pre_align_enabled', default_value='false'),
        DeclareLaunchArgument('lateral_shift_pre_align_mode', default_value='tag_center'),
        DeclareLaunchArgument('lateral_shift_pre_align_use_visual_feedback', default_value='false'),
        DeclareLaunchArgument('lateral_shift_continue_after_tag_loss', default_value='true'),
        DeclareLaunchArgument('lateral_shift_pre_align_timeout_sec', default_value='2.5'),
        DeclareLaunchArgument('lateral_shift_pre_align_max_visual_error_deg', default_value='6.0'),
        DeclareLaunchArgument('lateral_shift_pre_align_hold_sec', default_value='0.20'),
        DeclareLaunchArgument('lateral_shift_pre_align_tolerance_deg', default_value='6.0'),
        DeclareLaunchArgument('lateral_shift_yaw_deg', default_value='25.0'),
        DeclareLaunchArgument('lateral_shift_yaw_sign', default_value='-1.0'),
        DeclareLaunchArgument('lateral_shift_deadband_m', default_value='0.03'),
        DeclareLaunchArgument('lateral_shift_distance_scale', default_value='1.0'),
        DeclareLaunchArgument('lateral_shift_max_distance_m', default_value='0.80'),
        DeclareLaunchArgument('lateral_shift_use_euclidean_progress', default_value='true'),
        DeclareLaunchArgument('lateral_shift_final_yaw_gate_deg', default_value='8.0'),
        DeclareLaunchArgument('lateral_shift_duck_entry_yaw_gate_deg', default_value='2.0'),
        DeclareLaunchArgument('lateral_shift_duck_entry_hold_sec', default_value='0.30'),
        DeclareLaunchArgument('limit_bar_duck_sequence_enabled', default_value='false'),
        DeclareLaunchArgument('duck_prepare_mode', default_value='2'),
        DeclareLaunchArgument('duck_prepare_left_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_prepare_right_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_prepare_delay_sec', default_value='2.0'),
        DeclareLaunchArgument('duck_walk_mode', default_value='2'),
        DeclareLaunchArgument('duck_walk_left_norm', default_value='0.90'),
        DeclareLaunchArgument('duck_walk_right_norm', default_value='0.90'),
        DeclareLaunchArgument('duck_walk_yaw_correction_enabled', default_value='false'),
        DeclareLaunchArgument('duck_walk_yaw_gain_norm_per_rad', default_value='0.45'),
        DeclareLaunchArgument('duck_walk_yaw_deadband_deg', default_value='3.0'),
        DeclareLaunchArgument('duck_walk_yaw_max_delta_norm', default_value='0.18'),
        DeclareLaunchArgument('duck_walk_distance_m', default_value='0.85'),
        DeclareLaunchArgument('duck_walk_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument('duck_resume_mode', default_value='0'),
        DeclareLaunchArgument('duck_resume_left_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_resume_right_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_resume_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('near_approach_use_cmd_vel', default_value='true'),
        DeclareLaunchArgument('near_approach_distance_error_m', default_value='0.18'),
        DeclareLaunchArgument('step_override_forward_left_norm', default_value='0.70'),
        DeclareLaunchArgument('step_override_forward_right_norm', default_value='0.70'),
        DeclareLaunchArgument('step_override_forward_scale_enabled', default_value='false'),
        DeclareLaunchArgument('step_override_forward_min_scale', default_value='0.45'),
        DeclareLaunchArgument('step_override_forward_yaw_correction_enabled', default_value='false'),
        DeclareLaunchArgument('step_override_forward_yaw_gain_norm_per_radps', default_value='0.70'),
        DeclareLaunchArgument('step_override_forward_yaw_max_delta_norm', default_value='0.16'),
        DeclareLaunchArgument('step_override_turn_cw_left_norm', default_value='0.60'),
        DeclareLaunchArgument('step_override_turn_cw_right_norm', default_value='-0.30'),
        DeclareLaunchArgument('step_override_turn_ccw_left_norm', default_value='-0.35'),
        DeclareLaunchArgument('step_override_turn_ccw_right_norm', default_value='0.60'),
        DeclareLaunchArgument('step_override_turn_scale_enabled', default_value='false'),
        DeclareLaunchArgument('step_override_turn_min_scale', default_value='0.45'),
        DeclareLaunchArgument('step_override_duck_align_turn_enabled', default_value='false'),
        DeclareLaunchArgument('step_override_duck_align_cw_left_norm', default_value='0.08'),
        DeclareLaunchArgument('step_override_duck_align_cw_right_norm', default_value='-0.045'),
        DeclareLaunchArgument('step_override_duck_align_ccw_left_norm', default_value='-0.045'),
        DeclareLaunchArgument('step_override_duck_align_ccw_right_norm', default_value='0.08'),
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
            parameters=[apriltag_config],
            remappings=[
                ('image_rect', '/camera/color/image_raw'),
                ('camera_info', '/camera/color/camera_info'),
            ],
        ),

        Node(
            package='auto_nav_pkg',
            executable='apriltag_align_test_node',
            name='apriltag_align_test',
            output='screen',
            parameters=[
                config,
                {
                    'cmd_vel_topic': cmd_vel_topic,
                    'pose_topic': pose_topic,
                    'enable_control': ParameterValue(enable_control, value_type=bool),
                    'control_output_mode': control_output_mode,
                    'target_frame_id': target_frame_id,
                    'target_child_frame_id': target_child_frame_id,
                    'target_tag_id': ParameterValue(target_tag_id, value_type=int),
                    'target_offset_x_m': ParameterValue(target_offset_x_m, value_type=float),
                    'target_tag_x_m': ParameterValue(target_tag_x_m, value_type=float),
                    'robot_center_offset_x_m': ParameterValue(
                        robot_center_offset_x_m,
                        value_type=float,
                    ),
                    'alignment_mode': alignment_mode,
                    'center_control_sign': ParameterValue(
                        center_control_sign,
                        value_type=float,
                    ),
                    'pose_control_sign': ParameterValue(
                        pose_control_sign,
                        value_type=float,
                    ),
                    'max_abs_angular_z': ParameterValue(
                        max_abs_angular_z,
                        value_type=float,
                    ),
                    'yaw_deadband_deg': ParameterValue(
                        yaw_deadband_deg,
                        value_type=float,
                    ),
                    'enable_forward_approach': ParameterValue(
                        enable_forward_approach,
                        value_type=bool,
                    ),
                    'target_distance_m': ParameterValue(
                        target_distance_m,
                        value_type=float,
                    ),
                    'max_forward_speed_mps': ParameterValue(
                        max_forward_speed_mps,
                        value_type=float,
                    ),
                    'forward_max_yaw_error_deg': ParameterValue(
                        forward_max_yaw_error_deg,
                        value_type=float,
                    ),
                    'approach_requires_aligned': ParameterValue(
                        approach_requires_aligned,
                        value_type=bool,
                    ),
                    'bridge_lateral_calibration_enabled': ParameterValue(
                        bridge_lateral_calibration_enabled,
                        value_type=bool,
                    ),
                    'bridge_lateral_deadband_m': ParameterValue(
                        bridge_lateral_deadband_m,
                        value_type=float,
                    ),
                    'bridge_lateral_hold_sec': ParameterValue(
                        bridge_lateral_hold_sec,
                        value_type=float,
                    ),
                    'bridge_lateral_forward_speed_mps': ParameterValue(
                        bridge_lateral_forward_speed_mps,
                        value_type=float,
                    ),
                    'bridge_lateral_kp_radps_per_m': ParameterValue(
                        bridge_lateral_kp_radps_per_m,
                        value_type=float,
                    ),
                    'bridge_lateral_min_abs_angular_z': ParameterValue(
                        bridge_lateral_min_abs_angular_z,
                        value_type=float,
                    ),
                    'bridge_lateral_max_abs_angular_z': ParameterValue(
                        bridge_lateral_max_abs_angular_z,
                        value_type=float,
                    ),
                    'bridge_lateral_control_sign': ParameterValue(
                        bridge_lateral_control_sign,
                        value_type=float,
                    ),
                    'bridge_lateral_max_abs_error_m': ParameterValue(
                        bridge_lateral_max_abs_error_m,
                        value_type=float,
                    ),
                    'bridge_lateral_finish_after_hold': ParameterValue(
                        bridge_lateral_finish_after_hold,
                        value_type=bool,
                    ),
                    'stale_timeout_sec': ParameterValue(
                        stale_timeout_sec,
                        value_type=float,
                    ),
                    'pose_stale_timeout_sec': ParameterValue(
                        pose_stale_timeout_sec,
                        value_type=float,
                    ),
                    'lock_reference_on_tag_loss': ParameterValue(
                        lock_reference_on_tag_loss,
                        value_type=bool,
                    ),
                    'locked_reference_along_sign': ParameterValue(
                        locked_reference_along_sign,
                        value_type=float,
                    ),
                    'lock_reference_update_while_visible': ParameterValue(
                        lock_reference_update_while_visible,
                        value_type=bool,
                    ),
                    'lateral_shift_sequence_enabled': ParameterValue(
                        lateral_shift_sequence_enabled,
                        value_type=bool,
                    ),
                    'lateral_shift_pre_align_enabled': ParameterValue(
                        lateral_shift_pre_align_enabled,
                        value_type=bool,
                    ),
                    'lateral_shift_pre_align_mode': lateral_shift_pre_align_mode,
                    'lateral_shift_pre_align_use_visual_feedback': ParameterValue(
                        lateral_shift_pre_align_use_visual_feedback,
                        value_type=bool,
                    ),
                    'lateral_shift_continue_after_tag_loss': ParameterValue(
                        lateral_shift_continue_after_tag_loss,
                        value_type=bool,
                    ),
                    'lateral_shift_pre_align_timeout_sec': ParameterValue(
                        lateral_shift_pre_align_timeout_sec,
                        value_type=float,
                    ),
                    'lateral_shift_pre_align_max_visual_error_deg': ParameterValue(
                        lateral_shift_pre_align_max_visual_error_deg,
                        value_type=float,
                    ),
                    'lateral_shift_pre_align_hold_sec': ParameterValue(
                        lateral_shift_pre_align_hold_sec,
                        value_type=float,
                    ),
                    'lateral_shift_pre_align_tolerance_deg': ParameterValue(
                        lateral_shift_pre_align_tolerance_deg,
                        value_type=float,
                    ),
                    'lateral_shift_yaw_deg': ParameterValue(
                        lateral_shift_yaw_deg,
                        value_type=float,
                    ),
                    'lateral_shift_yaw_sign': ParameterValue(
                        lateral_shift_yaw_sign,
                        value_type=float,
                    ),
                    'lateral_shift_deadband_m': ParameterValue(
                        lateral_shift_deadband_m,
                        value_type=float,
                    ),
                    'lateral_shift_distance_scale': ParameterValue(
                        lateral_shift_distance_scale,
                        value_type=float,
                    ),
                    'lateral_shift_max_distance_m': ParameterValue(
                        lateral_shift_max_distance_m,
                        value_type=float,
                    ),
                    'lateral_shift_use_euclidean_progress': ParameterValue(
                        lateral_shift_use_euclidean_progress,
                        value_type=bool,
                    ),
                    'lateral_shift_final_yaw_gate_deg': ParameterValue(
                        lateral_shift_final_yaw_gate_deg,
                        value_type=float,
                    ),
                    'lateral_shift_duck_entry_yaw_gate_deg': ParameterValue(
                        lateral_shift_duck_entry_yaw_gate_deg,
                        value_type=float,
                    ),
                    'lateral_shift_duck_entry_hold_sec': ParameterValue(
                        lateral_shift_duck_entry_hold_sec,
                        value_type=float,
                    ),
                    'limit_bar_duck_sequence_enabled': ParameterValue(
                        limit_bar_duck_sequence_enabled,
                        value_type=bool,
                    ),
                    'duck_prepare_mode': ParameterValue(
                        duck_prepare_mode,
                        value_type=int,
                    ),
                    'duck_prepare_left_norm': ParameterValue(
                        duck_prepare_left_norm,
                        value_type=float,
                    ),
                    'duck_prepare_right_norm': ParameterValue(
                        duck_prepare_right_norm,
                        value_type=float,
                    ),
                    'duck_prepare_delay_sec': ParameterValue(
                        duck_prepare_delay_sec,
                        value_type=float,
                    ),
                    'duck_walk_mode': ParameterValue(
                        duck_walk_mode,
                        value_type=int,
                    ),
                    'duck_walk_left_norm': ParameterValue(
                        duck_walk_left_norm,
                        value_type=float,
                    ),
                    'duck_walk_right_norm': ParameterValue(
                        duck_walk_right_norm,
                        value_type=float,
                    ),
                    'duck_walk_yaw_correction_enabled': ParameterValue(
                        duck_walk_yaw_correction_enabled,
                        value_type=bool,
                    ),
                    'duck_walk_yaw_gain_norm_per_rad': ParameterValue(
                        duck_walk_yaw_gain_norm_per_rad,
                        value_type=float,
                    ),
                    'duck_walk_yaw_deadband_deg': ParameterValue(
                        duck_walk_yaw_deadband_deg,
                        value_type=float,
                    ),
                    'duck_walk_yaw_max_delta_norm': ParameterValue(
                        duck_walk_yaw_max_delta_norm,
                        value_type=float,
                    ),
                    'duck_walk_distance_m': ParameterValue(
                        duck_walk_distance_m,
                        value_type=float,
                    ),
                    'duck_walk_timeout_sec': ParameterValue(
                        duck_walk_timeout_sec,
                        value_type=float,
                    ),
                    'duck_resume_mode': ParameterValue(
                        duck_resume_mode,
                        value_type=int,
                    ),
                    'duck_resume_left_norm': ParameterValue(
                        duck_resume_left_norm,
                        value_type=float,
                    ),
                    'duck_resume_right_norm': ParameterValue(
                        duck_resume_right_norm,
                        value_type=float,
                    ),
                    'duck_resume_delay_sec': ParameterValue(
                        duck_resume_delay_sec,
                        value_type=float,
                    ),
                    'near_approach_use_cmd_vel': ParameterValue(
                        near_approach_use_cmd_vel,
                        value_type=bool,
                    ),
                    'near_approach_distance_error_m': ParameterValue(
                        near_approach_distance_error_m,
                        value_type=float,
                    ),
                    'step_override_forward_left_norm': ParameterValue(
                        step_override_forward_left_norm,
                        value_type=float,
                    ),
                    'step_override_forward_right_norm': ParameterValue(
                        step_override_forward_right_norm,
                        value_type=float,
                    ),
                    'step_override_forward_scale_enabled': ParameterValue(
                        step_override_forward_scale_enabled,
                        value_type=bool,
                    ),
                    'step_override_forward_min_scale': ParameterValue(
                        step_override_forward_min_scale,
                        value_type=float,
                    ),
                    'step_override_forward_yaw_correction_enabled': ParameterValue(
                        step_override_forward_yaw_correction_enabled,
                        value_type=bool,
                    ),
                    'step_override_forward_yaw_gain_norm_per_radps': ParameterValue(
                        step_override_forward_yaw_gain_norm_per_radps,
                        value_type=float,
                    ),
                    'step_override_forward_yaw_max_delta_norm': ParameterValue(
                        step_override_forward_yaw_max_delta_norm,
                        value_type=float,
                    ),
                    'step_override_turn_cw_left_norm': ParameterValue(
                        step_override_turn_cw_left_norm,
                        value_type=float,
                    ),
                    'step_override_turn_cw_right_norm': ParameterValue(
                        step_override_turn_cw_right_norm,
                        value_type=float,
                    ),
                    'step_override_turn_ccw_left_norm': ParameterValue(
                        step_override_turn_ccw_left_norm,
                        value_type=float,
                    ),
                    'step_override_turn_ccw_right_norm': ParameterValue(
                        step_override_turn_ccw_right_norm,
                        value_type=float,
                    ),
                    'step_override_turn_scale_enabled': ParameterValue(
                        step_override_turn_scale_enabled,
                        value_type=bool,
                    ),
                    'step_override_turn_min_scale': ParameterValue(
                        step_override_turn_min_scale,
                        value_type=float,
                    ),
                    'step_override_duck_align_turn_enabled': ParameterValue(
                        step_override_duck_align_turn_enabled,
                        value_type=bool,
                    ),
                    'step_override_duck_align_cw_left_norm': ParameterValue(
                        step_override_duck_align_cw_left_norm,
                        value_type=float,
                    ),
                    'step_override_duck_align_cw_right_norm': ParameterValue(
                        step_override_duck_align_cw_right_norm,
                        value_type=float,
                    ),
                    'step_override_duck_align_ccw_left_norm': ParameterValue(
                        step_override_duck_align_ccw_left_norm,
                        value_type=float,
                    ),
                    'step_override_duck_align_ccw_right_norm': ParameterValue(
                        step_override_duck_align_ccw_right_norm,
                        value_type=float,
                    ),
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
                    'step_override_topic': '/serial_step_override',
                    'step_override_timeout_sec': 0.35,
                    'odom_y_correction_enabled': False,
                    'path_lateral_correction_enabled': False,
                    'path_yaw_correction_enabled': False,
                    'crouch_yaw_correction_enabled': False,
                    # Keep the standalone AprilTag test on the same mode-0
                    # walking floor as the race stack so the robot does not
                    # crawl during the approach phase.
                    'min_output_step_norm_override_modes': [0, 6],
                    'min_output_step_norm_override_norms': [0.20, 0.18],
                    'uphill_segment_step_floor_enabled': False,
                    'uphill_pitch_step_floor_enabled': False,
                },
            ],
        ),
    ])
