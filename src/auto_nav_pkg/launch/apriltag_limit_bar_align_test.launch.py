from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_share = FindPackageShare('auto_nav_pkg')
    apriltag_share = FindPackageShare('apriltag_ros')

    enable_camera = LaunchConfiguration('enable_camera')
    enable_apriltag = LaunchConfiguration('enable_apriltag')
    enable_control = LaunchConfiguration('enable_control')
    enable_serial_bridge = LaunchConfiguration('enable_serial_bridge')
    control_output_mode = LaunchConfiguration('control_output_mode')
    serial_device = LaunchConfiguration('serial_device')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    pose_topic = LaunchConfiguration('pose_topic')
    target_offset_x_m = LaunchConfiguration('target_offset_x_m')
    robot_center_offset_x_m = LaunchConfiguration('robot_center_offset_x_m')
    center_control_sign = LaunchConfiguration('center_control_sign')
    pose_control_sign = LaunchConfiguration('pose_control_sign')
    enable_forward_approach = LaunchConfiguration('enable_forward_approach')
    target_distance_m = LaunchConfiguration('target_distance_m')
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
    forward_max_yaw_error_deg = LaunchConfiguration('forward_max_yaw_error_deg')
    yaw_deadband_deg = LaunchConfiguration('yaw_deadband_deg')
    approach_requires_aligned = LaunchConfiguration('approach_requires_aligned')
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
    step_override_forward_yaw_correction_enabled = LaunchConfiguration(
        'step_override_forward_yaw_correction_enabled'
    )
    step_override_forward_scale_enabled = LaunchConfiguration(
        'step_override_forward_scale_enabled'
    )
    step_override_forward_min_scale = LaunchConfiguration('step_override_forward_min_scale')
    step_override_forward_yaw_gain_norm_per_radps = LaunchConfiguration(
        'step_override_forward_yaw_gain_norm_per_radps'
    )
    step_override_forward_yaw_max_delta_norm = LaunchConfiguration(
        'step_override_forward_yaw_max_delta_norm'
    )

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_apriltag', default_value='true'),
        DeclareLaunchArgument('enable_control', default_value='false'),
        DeclareLaunchArgument('enable_serial_bridge', default_value='false'),
        DeclareLaunchArgument('control_output_mode', default_value='step_override'),
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/race_cmd_vel'),
        DeclareLaunchArgument('pose_topic', default_value='/Odometry'),
        DeclareLaunchArgument('target_offset_x_m', default_value='-0.605'),
        DeclareLaunchArgument('robot_center_offset_x_m', default_value='0.0'),
        DeclareLaunchArgument('center_control_sign', default_value='-1.0'),
        DeclareLaunchArgument('pose_control_sign', default_value='-1.0'),
        DeclareLaunchArgument('enable_forward_approach', default_value='true'),
        DeclareLaunchArgument('target_distance_m', default_value='0.45'),
        DeclareLaunchArgument('stale_timeout_sec', default_value='0.60'),
        DeclareLaunchArgument('pose_stale_timeout_sec', default_value='0.80'),
        DeclareLaunchArgument('lock_reference_on_tag_loss', default_value='true'),
        DeclareLaunchArgument('locked_reference_along_sign', default_value='-1.0'),
        DeclareLaunchArgument('lock_reference_update_while_visible', default_value='false'),
        DeclareLaunchArgument('lateral_shift_sequence_enabled', default_value='true'),
        DeclareLaunchArgument('lateral_shift_pre_align_enabled', default_value='true'),
        DeclareLaunchArgument('lateral_shift_pre_align_mode', default_value='pose_pitch'),
        DeclareLaunchArgument('lateral_shift_pre_align_use_visual_feedback', default_value='true'),
        DeclareLaunchArgument('lateral_shift_continue_after_tag_loss', default_value='true'),
        DeclareLaunchArgument('lateral_shift_pre_align_timeout_sec', default_value='8.0'),
        DeclareLaunchArgument('lateral_shift_pre_align_max_visual_error_deg', default_value='3.0'),
        DeclareLaunchArgument('lateral_shift_pre_align_hold_sec', default_value='0.10'),
        DeclareLaunchArgument('lateral_shift_pre_align_tolerance_deg', default_value='2.0'),
        DeclareLaunchArgument('lateral_shift_yaw_deg', default_value='30.0'),
        DeclareLaunchArgument('lateral_shift_yaw_sign', default_value='-1.0'),
        DeclareLaunchArgument('lateral_shift_deadband_m', default_value='0.03'),
        DeclareLaunchArgument('lateral_shift_distance_scale', default_value='0.90'),
        DeclareLaunchArgument('lateral_shift_max_distance_m', default_value='0.95'),
        DeclareLaunchArgument('lateral_shift_use_euclidean_progress', default_value='true'),
        DeclareLaunchArgument('lateral_shift_final_yaw_gate_deg', default_value='6.0'),
        DeclareLaunchArgument('lateral_shift_duck_entry_yaw_gate_deg', default_value='1.0'),
        DeclareLaunchArgument('lateral_shift_duck_entry_hold_sec', default_value='0.30'),
        DeclareLaunchArgument('forward_max_yaw_error_deg', default_value='8.0'),
        DeclareLaunchArgument('yaw_deadband_deg', default_value='1.5'),
        DeclareLaunchArgument('approach_requires_aligned', default_value='true'),
        DeclareLaunchArgument('step_override_turn_cw_left_norm', default_value='0.22'),
        DeclareLaunchArgument('step_override_turn_cw_right_norm', default_value='-0.12'),
        DeclareLaunchArgument('step_override_turn_ccw_left_norm', default_value='-0.12'),
        DeclareLaunchArgument('step_override_turn_ccw_right_norm', default_value='0.22'),
        DeclareLaunchArgument('step_override_turn_scale_enabled', default_value='true'),
        DeclareLaunchArgument('step_override_turn_min_scale', default_value='0.45'),
        DeclareLaunchArgument('step_override_duck_align_turn_enabled', default_value='true'),
        DeclareLaunchArgument('step_override_duck_align_cw_left_norm', default_value='0.060'),
        DeclareLaunchArgument('step_override_duck_align_cw_right_norm', default_value='-0.035'),
        DeclareLaunchArgument('step_override_duck_align_ccw_left_norm', default_value='-0.035'),
        DeclareLaunchArgument('step_override_duck_align_ccw_right_norm', default_value='0.060'),
        DeclareLaunchArgument('limit_bar_duck_sequence_enabled', default_value='true'),
        DeclareLaunchArgument('duck_prepare_mode', default_value='2'),
        DeclareLaunchArgument('duck_prepare_left_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_prepare_right_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_prepare_delay_sec', default_value='2.0'),
        DeclareLaunchArgument('duck_walk_mode', default_value='2'),
        DeclareLaunchArgument('duck_walk_left_norm', default_value='0.90'),
        DeclareLaunchArgument('duck_walk_right_norm', default_value='0.90'),
        DeclareLaunchArgument('duck_walk_yaw_correction_enabled', default_value='true'),
        DeclareLaunchArgument('duck_walk_yaw_gain_norm_per_rad', default_value='1.20'),
        DeclareLaunchArgument('duck_walk_yaw_deadband_deg', default_value='0.5'),
        DeclareLaunchArgument('duck_walk_yaw_max_delta_norm', default_value='0.30'),
        DeclareLaunchArgument('duck_walk_distance_m', default_value='0.85'),
        DeclareLaunchArgument('duck_walk_timeout_sec', default_value='14.0'),
        DeclareLaunchArgument('duck_resume_mode', default_value='0'),
        DeclareLaunchArgument('duck_resume_left_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_resume_right_norm', default_value='0.0'),
        DeclareLaunchArgument('duck_resume_delay_sec', default_value='1.0'),
        DeclareLaunchArgument('step_override_forward_scale_enabled', default_value='true'),
        DeclareLaunchArgument('step_override_forward_min_scale', default_value='0.35'),
        DeclareLaunchArgument('step_override_forward_yaw_correction_enabled', default_value='true'),
        DeclareLaunchArgument('step_override_forward_yaw_gain_norm_per_radps', default_value='0.90'),
        DeclareLaunchArgument('step_override_forward_yaw_max_delta_norm', default_value='0.18'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    auto_share,
                    'launch',
                    'apriltag_align_test.launch.py',
                ])
            ),
            launch_arguments={
                'apriltag_config': PathJoinSubstitution([
                    apriltag_share,
                    'cfg',
                    'tags_36h11_limit_bar_right_test.yaml',
                ]),
                'target_frame_id': 'camera_color_optical_frame',
                'target_child_frame_id': 'limit_bar_right',
                'target_tag_id': '12',
                'target_offset_x_m': target_offset_x_m,
                'robot_center_offset_x_m': robot_center_offset_x_m,
                'alignment_mode': 'center_x_offset',
                'center_control_sign': center_control_sign,
                'pose_control_sign': pose_control_sign,
                'enable_control': enable_control,
                'enable_serial_bridge': enable_serial_bridge,
                'control_output_mode': control_output_mode,
                'enable_camera': enable_camera,
                'enable_apriltag': enable_apriltag,
                'serial_device': serial_device,
                'cmd_vel_topic': cmd_vel_topic,
                'pose_topic': pose_topic,
                'enable_forward_approach': enable_forward_approach,
                'target_distance_m': target_distance_m,
                'stale_timeout_sec': stale_timeout_sec,
                'pose_stale_timeout_sec': pose_stale_timeout_sec,
                'lock_reference_on_tag_loss': lock_reference_on_tag_loss,
                'locked_reference_along_sign': locked_reference_along_sign,
                'lock_reference_update_while_visible': lock_reference_update_while_visible,
                'lateral_shift_sequence_enabled': lateral_shift_sequence_enabled,
                'lateral_shift_pre_align_enabled': lateral_shift_pre_align_enabled,
                'lateral_shift_pre_align_mode': lateral_shift_pre_align_mode,
                'lateral_shift_pre_align_use_visual_feedback': (
                    lateral_shift_pre_align_use_visual_feedback
                ),
                'lateral_shift_continue_after_tag_loss': lateral_shift_continue_after_tag_loss,
                'lateral_shift_pre_align_timeout_sec': lateral_shift_pre_align_timeout_sec,
                'lateral_shift_pre_align_max_visual_error_deg': (
                    lateral_shift_pre_align_max_visual_error_deg
                ),
                'lateral_shift_pre_align_hold_sec': lateral_shift_pre_align_hold_sec,
                'lateral_shift_pre_align_tolerance_deg': lateral_shift_pre_align_tolerance_deg,
                'lateral_shift_yaw_deg': lateral_shift_yaw_deg,
                'lateral_shift_yaw_sign': lateral_shift_yaw_sign,
                'lateral_shift_deadband_m': lateral_shift_deadband_m,
                'lateral_shift_distance_scale': lateral_shift_distance_scale,
                'lateral_shift_max_distance_m': lateral_shift_max_distance_m,
                'lateral_shift_use_euclidean_progress': lateral_shift_use_euclidean_progress,
                'lateral_shift_final_yaw_gate_deg': lateral_shift_final_yaw_gate_deg,
                'lateral_shift_duck_entry_yaw_gate_deg': lateral_shift_duck_entry_yaw_gate_deg,
                'lateral_shift_duck_entry_hold_sec': lateral_shift_duck_entry_hold_sec,
                'forward_max_yaw_error_deg': forward_max_yaw_error_deg,
                'yaw_deadband_deg': yaw_deadband_deg,
                'approach_requires_aligned': approach_requires_aligned,
                'step_override_turn_cw_left_norm': step_override_turn_cw_left_norm,
                'step_override_turn_cw_right_norm': step_override_turn_cw_right_norm,
                'step_override_turn_ccw_left_norm': step_override_turn_ccw_left_norm,
                'step_override_turn_ccw_right_norm': step_override_turn_ccw_right_norm,
                'step_override_turn_scale_enabled': step_override_turn_scale_enabled,
                'step_override_turn_min_scale': step_override_turn_min_scale,
                'step_override_duck_align_turn_enabled': step_override_duck_align_turn_enabled,
                'step_override_duck_align_cw_left_norm': step_override_duck_align_cw_left_norm,
                'step_override_duck_align_cw_right_norm': step_override_duck_align_cw_right_norm,
                'step_override_duck_align_ccw_left_norm': step_override_duck_align_ccw_left_norm,
                'step_override_duck_align_ccw_right_norm': step_override_duck_align_ccw_right_norm,
                'limit_bar_duck_sequence_enabled': limit_bar_duck_sequence_enabled,
                'duck_prepare_mode': duck_prepare_mode,
                'duck_prepare_left_norm': duck_prepare_left_norm,
                'duck_prepare_right_norm': duck_prepare_right_norm,
                'duck_prepare_delay_sec': duck_prepare_delay_sec,
                'duck_walk_mode': duck_walk_mode,
                'duck_walk_left_norm': duck_walk_left_norm,
                'duck_walk_right_norm': duck_walk_right_norm,
                'duck_walk_yaw_correction_enabled': duck_walk_yaw_correction_enabled,
                'duck_walk_yaw_gain_norm_per_rad': duck_walk_yaw_gain_norm_per_rad,
                'duck_walk_yaw_deadband_deg': duck_walk_yaw_deadband_deg,
                'duck_walk_yaw_max_delta_norm': duck_walk_yaw_max_delta_norm,
                'duck_walk_distance_m': duck_walk_distance_m,
                'duck_walk_timeout_sec': duck_walk_timeout_sec,
                'duck_resume_mode': duck_resume_mode,
                'duck_resume_left_norm': duck_resume_left_norm,
                'duck_resume_right_norm': duck_resume_right_norm,
                'duck_resume_delay_sec': duck_resume_delay_sec,
                'step_override_forward_yaw_correction_enabled': (
                    step_override_forward_yaw_correction_enabled
                ),
                'step_override_forward_scale_enabled': step_override_forward_scale_enabled,
                'step_override_forward_min_scale': step_override_forward_min_scale,
                'step_override_forward_yaw_gain_norm_per_radps': (
                    step_override_forward_yaw_gain_norm_per_radps
                ),
                'step_override_forward_yaw_max_delta_norm': step_override_forward_yaw_max_delta_norm,
            }.items(),
        ),
    ])
