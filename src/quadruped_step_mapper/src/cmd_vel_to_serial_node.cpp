#include "quadruped_step_mapper/cmd_vel_to_serial_node.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr const char * kAnsiReset = "\033[0m";
constexpr const char * kAnsiGreen = "\033[1;32m";
constexpr const char * kAnsiYellow = "\033[1;33m";
constexpr const char * kAnsiCyan = "\033[1;36m";
constexpr const char * kAnsiMagenta = "\033[1;35m";
constexpr const char * kAnsiBlue = "\033[0;94m";
constexpr const char * kAnsiSerial = "\033[0;96m";

const char * styleOrEmpty(const char * style)
{
  return std::getenv("NO_COLOR") == nullptr ? style : "";
}

const char * resetOrEmpty()
{
  return std::getenv("NO_COLOR") == nullptr ? kAnsiReset : "";
}

double normalizeAngle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double rollFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  return std::atan2(sinr_cosp, cosr_cosp);
}

}  // namespace

namespace quadruped_step_mapper
{

CmdVelToSerialNode::CmdVelToSerialNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("cmd_vel_to_serial", options),
  last_cmd_stamp_(this->get_clock()->now()),
  last_open_attempt_(this->get_clock()->now()),
  last_serial_log_time_(this->get_clock()->now()),
  last_step_log_time_(this->get_clock()->now())
{
  const auto input_cmd_vel_topic =
    this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");

  mapping_parameters_.step_frequency_hz =
    this->declare_parameter<double>("step_frequency_hz", 2.5);
  mapping_parameters_.stance_half_width_m =
    this->declare_parameter<double>("stance_half_width_m", 0.18);
  mapping_parameters_.max_step_length_m =
    this->declare_parameter<double>("max_step_length_m", 0.10);
  mapping_parameters_.max_step_yaw_rad =
    this->declare_parameter<double>("max_step_yaw_rad", 0.35);
  mapping_parameters_.max_linear_speed_mps =
    this->declare_parameter<double>("max_linear_speed_mps", 0.40);
  mapping_parameters_.max_angular_speed_rps =
    this->declare_parameter<double>("max_angular_speed_rps", 1.20);
  mapping_parameters_.min_effective_linear_speed_mps =
    this->declare_parameter<double>("min_effective_linear_speed_mps", 0.01);
  mapping_parameters_.min_effective_angular_speed_rps =
    this->declare_parameter<double>("min_effective_angular_speed_rps", 0.05);
  mapping_parameters_.scale_to_limits =
    this->declare_parameter<bool>("scale_to_limits", true);

  serial_device_ = this->declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
  baud_rate_ = this->declare_parameter<int>("baud_rate", 115200);
  reconnect_interval_sec_ = this->declare_parameter<double>("reconnect_interval_sec", 1.0);
  transmit_rate_hz_ = this->declare_parameter<double>("transmit_rate_hz", 27.0);
  cmd_vel_timeout_sec_ = this->declare_parameter<double>("cmd_vel_timeout_sec", 0.50);
  mode_ = this->declare_parameter<int>("mode", 0);
  fixed_step_override_enabled_ = this->declare_parameter<bool>("fixed_step_override_enabled", false);
  fixed_step_override_mode_ = this->declare_parameter<int>("fixed_step_override_mode", 1);
  fixed_step_override_left_norm_ = std::clamp(
    this->declare_parameter<double>("fixed_step_override_left_norm", 1.0), -1.0, 1.0);
  fixed_step_override_right_norm_ = std::clamp(
    this->declare_parameter<double>("fixed_step_override_right_norm", 1.0), -1.0, 1.0);
  const auto fixed_step_override_modes = this->declare_parameter<std::vector<int64_t>>(
    "fixed_step_override_modes", std::vector<int64_t>{});
  const auto fixed_step_override_left_norms = this->declare_parameter<std::vector<double>>(
    "fixed_step_override_left_norms", std::vector<double>{});
  const auto fixed_step_override_right_norms = this->declare_parameter<std::vector<double>>(
    "fixed_step_override_right_norms", std::vector<double>{});
  goal_topic_ = this->declare_parameter<std::string>("goal_topic", "/goal_pose");
  path_reference_start_topic_ =
    this->declare_parameter<std::string>("path_reference_start_topic", "/serial_path_reference_start");
  path_reference_topic_ =
    this->declare_parameter<std::string>("path_reference_topic", "/serial_path_reference");
  path_reference_start_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("path_reference_start_timeout_sec", 0.50));
  odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");
  pose_yaw_offset_rad_ =
    this->declare_parameter<double>("pose_yaw_offset_deg", 0.0) * kPi / 180.0;
  odom_y_correction_enabled_ = this->declare_parameter<bool>("odom_y_correction_enabled", false);
  odom_y_correction_use_goal_y_ =
    this->declare_parameter<bool>("odom_y_correction_use_goal_y", true);
  odom_y_correction_modes_ =
    this->declare_parameter<std::vector<int64_t>>("odom_y_correction_modes", std::vector<int64_t>{1});
  odom_y_correction_deadband_m_ =
    std::max(0.0, this->declare_parameter<double>("odom_y_correction_deadband_m", 0.05));
  odom_y_correction_gain_norm_per_m_ =
    std::max(0.0, this->declare_parameter<double>("odom_y_correction_gain_norm_per_m", 0.0));
  odom_y_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("odom_y_correction_max_norm", 0.0), 0.0, 1.0);
  odom_y_correction_positive_only_ =
    this->declare_parameter<bool>("odom_y_correction_positive_only", true);
  odom_y_correction_stale_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("odom_y_correction_stale_timeout_sec", 0.50));
  odom_y_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("odom_y_correction_goal_distance_min_m", 0.25));
  odom_y_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("odom_y_correction_max_abs_wz_rps", 0.35));
  path_lateral_correction_enabled_ =
    this->declare_parameter<bool>("path_lateral_correction_enabled", false);
  path_lateral_correction_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "path_lateral_correction_modes", std::vector<int64_t>{0});
  path_lateral_direct_override_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "path_lateral_direct_override_modes", std::vector<int64_t>{});
  path_lateral_correction_deadband_m_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_deadband_m", 0.05));
  path_lateral_correction_gain_norm_per_m_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_gain_norm_per_m", 0.0));
  path_lateral_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("path_lateral_correction_max_norm", 0.0), 0.0, 1.0);
  path_lateral_correction_sign_ =
    this->declare_parameter<double>("path_lateral_correction_sign", 1.0) >= 0.0 ? 1.0 : -1.0;
  path_lateral_direct_high_norm_ = std::clamp(
    this->declare_parameter<double>("path_lateral_direct_high_norm", 1.0), -1.0, 1.0);
  path_lateral_direct_low_norm_ = std::clamp(
    this->declare_parameter<double>("path_lateral_direct_low_norm", 0.2), -1.0, 1.0);
  path_lateral_correction_stale_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_stale_timeout_sec", 0.50));
  path_lateral_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_goal_distance_min_m", 0.25));
  path_lateral_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_max_abs_wz_rps", 0.35));
  path_lateral_correction_min_segment_length_m_ = std::max(
    0.0, this->declare_parameter<double>("path_lateral_correction_min_segment_length_m", 0.30));
  uphill_path_lateral_correction_enabled_ =
    this->declare_parameter<bool>("uphill_path_lateral_correction_enabled", false);
  uphill_path_lateral_correction_deadband_m_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_lateral_correction_deadband_m", 0.03));
  uphill_path_lateral_correction_gain_norm_per_m_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_lateral_correction_gain_norm_per_m", 0.0));
  uphill_path_lateral_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("uphill_path_lateral_correction_max_norm", 0.0), 0.0, 1.0);
  uphill_path_lateral_correction_sign_ =
    this->declare_parameter<double>("uphill_path_lateral_correction_sign", 1.0) >= 0.0 ? 1.0 : -1.0;
  uphill_path_lateral_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_lateral_correction_goal_distance_min_m", 0.15));
  uphill_path_lateral_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_lateral_correction_max_abs_wz_rps", 0.80));
  path_yaw_correction_enabled_ =
    this->declare_parameter<bool>("path_yaw_correction_enabled", false);
  path_yaw_correction_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "path_yaw_correction_modes", std::vector<int64_t>{0});
  path_yaw_direct_override_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "path_yaw_direct_override_modes", std::vector<int64_t>{});
  path_yaw_correction_deadband_rad_ =
    std::max(
      0.0,
      this->declare_parameter<double>("path_yaw_correction_deadband_deg", 5.0) * kPi / 180.0);
  path_yaw_correction_gain_norm_per_rad_ = std::max(
    0.0, this->declare_parameter<double>("path_yaw_correction_gain_norm_per_rad", 0.0));
  path_yaw_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("path_yaw_correction_max_norm", 0.0), 0.0, 1.0);
  path_yaw_direct_high_norm_ = std::clamp(
    this->declare_parameter<double>("path_yaw_direct_high_norm", 1.0), -1.0, 1.0);
  path_yaw_direct_low_norm_ = std::clamp(
    this->declare_parameter<double>("path_yaw_direct_low_norm", 0.2), -1.0, 1.0);
  path_yaw_correction_stale_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("path_yaw_correction_stale_timeout_sec", 0.50));
  path_yaw_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("path_yaw_correction_goal_distance_min_m", 0.35));
  path_yaw_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("path_yaw_correction_max_abs_wz_rps", 0.45));
  path_yaw_correction_min_segment_length_m_ = std::max(
    0.0, this->declare_parameter<double>("path_yaw_correction_min_segment_length_m", 0.50));
  path_correction_profile_topic_ = this->declare_parameter<std::string>(
    "path_correction_profile_topic", "/race_manager/path_correction_profile");
  // 2026-05-01: old behavior was effectively "normal correction everywhere".
  // Keep that as the default, while allowing obstacle_manager to lock profiles per waypoint interval.
  // Profiles: 0=off, 1=normal, 2=uphill_strong, 3=uphill_strong_sign_test.
  path_correction_profile_ = std::clamp(
    static_cast<int>(this->declare_parameter<int>("default_path_correction_profile", 1)), 0, 3);
  path_correction_new_goal_delay_sec_ = std::max(
    0.0, this->declare_parameter<double>("path_correction_new_goal_delay_sec", 0.30));
  uphill_path_yaw_correction_enabled_ =
    this->declare_parameter<bool>("uphill_path_yaw_correction_enabled", false);
  uphill_path_yaw_correction_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "uphill_path_yaw_correction_modes", std::vector<int64_t>{0});
  uphill_path_yaw_correction_deadband_rad_ =
    std::max(
      0.0,
      this->declare_parameter<double>("uphill_path_yaw_correction_deadband_deg", 3.0) * kPi / 180.0);
  uphill_path_yaw_correction_gain_norm_per_rad_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_yaw_correction_gain_norm_per_rad", 0.0));
  uphill_path_yaw_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("uphill_path_yaw_correction_max_norm", 0.0), 0.0, 1.0);
  uphill_path_yaw_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_yaw_correction_goal_distance_min_m", 0.20));
  uphill_path_yaw_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("uphill_path_yaw_correction_max_abs_wz_rps", 0.80));
  crouch_yaw_correction_enabled_ =
    this->declare_parameter<bool>("crouch_yaw_correction_enabled", false);
  crouch_yaw_correction_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "crouch_yaw_correction_modes", std::vector<int64_t>{2});
  crouch_yaw_correction_deadband_rad_ =
    std::max(
      0.0,
      this->declare_parameter<double>("crouch_yaw_correction_deadband_deg", 3.0) * kPi / 180.0);
  crouch_yaw_correction_gain_norm_per_rad_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_correction_gain_norm_per_rad", 0.0));
  crouch_yaw_correction_max_norm_ = std::clamp(
    this->declare_parameter<double>("crouch_yaw_correction_max_norm", 0.0), 0.0, 1.0);
  crouch_yaw_initial_boost_enabled_ =
    this->declare_parameter<bool>("crouch_yaw_initial_boost_enabled", false);
  crouch_yaw_initial_boost_duration_sec_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_initial_boost_duration_sec", 0.0));
  crouch_yaw_initial_boost_progress_m_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_initial_boost_progress_m", 0.0));
  crouch_yaw_initial_boost_min_error_rad_ = std::max(
    0.0,
    this->declare_parameter<double>("crouch_yaw_initial_boost_min_error_deg", 0.0) * kPi / 180.0);
  crouch_yaw_initial_boost_max_norm_ = std::clamp(
    this->declare_parameter<double>("crouch_yaw_initial_boost_max_norm", 0.0), 0.0, 1.0);
  crouch_yaw_lateral_priority_enabled_ =
    this->declare_parameter<bool>("crouch_yaw_lateral_priority_enabled", false);
  crouch_yaw_lateral_priority_error_threshold_m_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_lateral_priority_error_threshold_m", 0.07));
  crouch_yaw_lateral_priority_max_norm_ = std::clamp(
    this->declare_parameter<double>("crouch_yaw_lateral_priority_max_norm", 0.03), 0.0, 1.0);
  crouch_yaw_correction_stale_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_correction_stale_timeout_sec", 0.50));
  crouch_yaw_correction_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_correction_goal_distance_min_m", 0.25));
  crouch_yaw_correction_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("crouch_yaw_correction_max_abs_wz_rps", 0.35));
  crouch_forward_bias_enabled_ =
    this->declare_parameter<bool>("crouch_forward_bias_enabled", false);
  crouch_forward_bias_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "crouch_forward_bias_modes", std::vector<int64_t>{2});
  crouch_forward_bias_norm_ = std::clamp(
    this->declare_parameter<double>("crouch_forward_bias_norm", 0.0), 0.0, 1.0);
  crouch_forward_bias_min_forward_step_norm_ = std::clamp(
    this->declare_parameter<double>("crouch_forward_bias_min_forward_step_norm", 0.0), 0.0, 1.0);
  crouch_forward_bias_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("crouch_forward_bias_max_abs_wz_rps", 0.35));
  same_direction_turn_enabled_ = this->declare_parameter<bool>("same_direction_turn_enabled", false);
  same_direction_turn_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "same_direction_turn_modes", std::vector<int64_t>{0});
  same_direction_turn_min_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("same_direction_turn_min_wz_rps", 0.15));
  same_direction_turn_min_norm_ = std::clamp(
    this->declare_parameter<double>("same_direction_turn_min_norm", 0.02), 0.0, 1.0);
  opposite_sign_turn_shape_enabled_ =
    this->declare_parameter<bool>("opposite_sign_turn_shape_enabled", false);
  opposite_sign_turn_shape_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "opposite_sign_turn_shape_modes", std::vector<int64_t>{6});
  opposite_sign_turn_shape_small_norm_ = std::clamp(
    this->declare_parameter<double>("opposite_sign_turn_shape_small_norm", 0.20), 0.0, 1.0);
  opposite_sign_turn_shape_large_norm_ = std::clamp(
    this->declare_parameter<double>("opposite_sign_turn_shape_large_norm", 0.65), 0.0, 1.0);
  step_override_small_opposite_sign_shape_enabled_ =
    this->declare_parameter<bool>("step_override_small_opposite_sign_shape_enabled", false);
  step_override_small_opposite_sign_shape_modes_ =
    this->declare_parameter<std::vector<int64_t>>(
    "step_override_small_opposite_sign_shape_modes",
    std::vector<int64_t>{0});
  step_override_small_opposite_sign_shape_small_norm_ = std::clamp(
    this->declare_parameter<double>("step_override_small_opposite_sign_shape_small_norm", 0.15),
    0.0,
    1.0);
  step_override_small_opposite_sign_shape_large_norm_ = std::clamp(
    this->declare_parameter<double>("step_override_small_opposite_sign_shape_large_norm", 0.20),
    0.0,
    1.0);
  cmd_vel_small_opposite_sign_shape_enabled_ =
    this->declare_parameter<bool>("cmd_vel_small_opposite_sign_shape_enabled", false);
  cmd_vel_small_opposite_sign_shape_modes_ =
    this->declare_parameter<std::vector<int64_t>>(
    "cmd_vel_small_opposite_sign_shape_modes",
    std::vector<int64_t>{0});
  cmd_vel_small_opposite_sign_shape_small_norm_ = std::clamp(
    this->declare_parameter<double>("cmd_vel_small_opposite_sign_shape_small_norm", 0.18),
    0.0,
    1.0);
  cmd_vel_small_opposite_sign_shape_large_norm_ = std::clamp(
    this->declare_parameter<double>("cmd_vel_small_opposite_sign_shape_large_norm", 0.20),
    0.0,
    1.0);
  min_output_step_norm_enabled_ =
    this->declare_parameter<bool>("min_output_step_norm_enabled", false);
  min_output_step_norm_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "min_output_step_norm_modes", std::vector<int64_t>{2, 5});
  min_output_step_norm_ = std::clamp(
    this->declare_parameter<double>("min_output_step_norm", 0.0), 0.0, 1.0);
  min_output_step_norm_override_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "min_output_step_norm_override_modes", std::vector<int64_t>{});
  auto min_output_step_norm_override_norms = this->declare_parameter<std::vector<double>>(
    "min_output_step_norm_override_norms", std::vector<double>{});
  if (min_output_step_norm_override_modes_.size() != min_output_step_norm_override_norms.size()) {
    throw std::runtime_error(
      "min_output_step_norm_override_modes and min_output_step_norm_override_norms "
      "must have the same length.");
  }
  min_output_step_norm_override_norms_.reserve(min_output_step_norm_override_norms.size());
  for (const auto norm : min_output_step_norm_override_norms) {
    min_output_step_norm_override_norms_.push_back(std::clamp(norm, 0.0, 1.0));
  }
  uphill_segment_step_floor_enabled_ =
    this->declare_parameter<bool>("uphill_segment_step_floor_enabled", false);
  uphill_segment_step_floor_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "uphill_segment_step_floor_modes", std::vector<int64_t>{0});
  uphill_segment_step_floor_forward_only_ =
    this->declare_parameter<bool>("uphill_segment_step_floor_forward_only", true);
  uphill_segment_step_floor_min_norm_ = std::clamp(
    this->declare_parameter<double>("uphill_segment_step_floor_min_norm", 0.0), 0.0, 1.0);
  uphill_segment_step_floor_goal_distance_min_m_ = std::max(
    0.0, this->declare_parameter<double>("uphill_segment_step_floor_goal_distance_min_m", 0.0));
  uphill_segment_step_floor_stale_timeout_sec_ = std::max(
    0.0,
    this->declare_parameter<double>("uphill_segment_step_floor_stale_timeout_sec", 0.50));
  uphill_segment_step_floor_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("uphill_segment_step_floor_max_abs_wz_rps", 0.0));
  uphill_pitch_step_floor_enabled_ =
    this->declare_parameter<bool>("uphill_pitch_step_floor_enabled", false);
  uphill_pitch_step_floor_require_segment_enable_ =
    this->declare_parameter<bool>("uphill_pitch_step_floor_require_segment_enable", false);
  uphill_pitch_step_floor_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "uphill_pitch_step_floor_modes", std::vector<int64_t>{0});
  uphill_pitch_step_floor_min_pitch_rad_ = std::max(
    0.0,
    this->declare_parameter<double>("uphill_pitch_step_floor_min_pitch_deg", 8.0) * kPi / 180.0);
  uphill_pitch_step_floor_use_abs_pitch_ =
    this->declare_parameter<bool>("uphill_pitch_step_floor_use_abs_pitch", true);
  uphill_pitch_step_floor_forward_only_ =
    this->declare_parameter<bool>("uphill_pitch_step_floor_forward_only", true);
  uphill_pitch_step_floor_min_norm_ = std::clamp(
    this->declare_parameter<double>("uphill_pitch_step_floor_min_norm", 0.75), 0.0, 1.0);
  uphill_pitch_step_floor_stale_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("uphill_pitch_step_floor_stale_timeout_sec", 0.50));
  uphill_pitch_step_floor_max_abs_wz_rps_ = std::max(
    0.0, this->declare_parameter<double>("uphill_pitch_step_floor_max_abs_wz_rps", 0.35));
  step_override_topic_ = this->declare_parameter<std::string>(
    "step_override_topic", "/serial_step_override");
  step_once_topic_ = this->declare_parameter<std::string>(
    "step_once_topic", "/serial_step_once");
  uphill_step_floor_state_topic_ = this->declare_parameter<std::string>(
    "uphill_step_floor_state_topic", "/race_manager/uphill_step_floor_enabled");
  fixed_step_override_state_topic_ = this->declare_parameter<std::string>(
    "fixed_step_override_state_topic", "/motion_bridge/fixed_step_override_enabled");
  step_override_timeout_sec_ = std::max(
    0.0, this->declare_parameter<double>("step_override_timeout_sec", 0.25));

  mode_topic_ = this->declare_parameter<std::string>("mode_topic", "/dog_mode_current");
  mode_echo_topic_ =
    this->declare_parameter<std::string>("mode_echo_topic", "/dog_mode_feedback/current");
  serial_connected_topic_ =
    this->declare_parameter<std::string>("serial_connected_topic", "/motion_bridge/serial_connected");
  start_signal_rx_enabled_ =
    this->declare_parameter<bool>("start_signal_rx_enabled", false);
  start_signal_topic_ =
    this->declare_parameter<std::string>("start_signal_topic", "/motion_bridge/start_signal");
  start_signal_token_ =
    this->declare_parameter<std::string>("start_signal_token", "start");
  finish_signal_topic_ =
    this->declare_parameter<std::string>("finish_signal_topic", "/motion_bridge/finish_signal");
  finish_signal_token_ =
    this->declare_parameter<std::string>("finish_signal_token", "finish");
  action_ack_topic_ =
    this->declare_parameter<std::string>("action_ack_topic", "/motion_bridge/action_ack");
  jumping_signal_token_ =
    this->declare_parameter<std::string>("jumping_signal_token", "jumping");
  start_signal_case_sensitive_ =
    this->declare_parameter<bool>("start_signal_case_sensitive", false);
  log_serial_rx_frames_ =
    this->declare_parameter<bool>("log_serial_rx_frames", false);
  const auto serial_rx_max_frame_len_param =
    this->declare_parameter<int>("serial_rx_max_frame_len", 128);
  serial_rx_max_frame_len_ = static_cast<std::size_t>(
    serial_rx_max_frame_len_param < 2 ? 2 : serial_rx_max_frame_len_param);
  output_precision_ = this->declare_parameter<int>("output_precision", 3);
  send_zero_on_timeout_ = this->declare_parameter<bool>("send_zero_on_timeout", true);
  log_serial_messages_ = this->declare_parameter<bool>("log_serial_messages", false);
  log_step_debug_ = this->declare_parameter<bool>("log_step_debug", true);
  output_step_slew_limit_enabled_ =
    this->declare_parameter<bool>("output_step_slew_limit_enabled", false);
  output_step_slew_limit_modes_ = this->declare_parameter<std::vector<int64_t>>(
    "output_step_slew_limit_modes", std::vector<int64_t>{});
  output_step_slew_limit_max_delta_norm_ = std::max(
    0.0, this->declare_parameter<double>("output_step_slew_limit_max_delta_norm", 0.0));
  serial_log_min_interval_sec_ = this->declare_parameter<double>("serial_log_min_interval_sec", 1.0);
  step_debug_log_interval_sec_ = this->declare_parameter<double>("step_debug_log_interval_sec", 1.0);
  serial_log_change_threshold_ = this->declare_parameter<double>("serial_log_change_threshold", 0.02);
  step_log_change_threshold_ = this->declare_parameter<double>("step_log_change_threshold", 0.02);

  message_prefix_ = this->declare_parameter<std::string>("message_prefix", "[");
  field_separator_ = this->declare_parameter<std::string>("field_separator", ",");
  message_suffix_ = this->declare_parameter<std::string>("message_suffix", "]\n");

  if (transmit_rate_hz_ <= 0.0) {
    throw std::runtime_error("transmit_rate_hz must be > 0.");
  }
  if (cmd_vel_timeout_sec_ < 0.0) {
    throw std::runtime_error("cmd_vel_timeout_sec must be >= 0.");
  }
  if (reconnect_interval_sec_ < 0.0) {
    throw std::runtime_error("reconnect_interval_sec must be >= 0.");
  }
  if (output_precision_ < 0) {
    throw std::runtime_error("output_precision must be >= 0.");
  }

  mapper_ = std::make_unique<StepLengthMapper>(mapping_parameters_);
  if (fixed_step_override_enabled_) {
    const bool has_multi_mode_config =
      !fixed_step_override_modes.empty() ||
      !fixed_step_override_left_norms.empty() ||
      !fixed_step_override_right_norms.empty();
    if (has_multi_mode_config) {
      if (
        fixed_step_override_modes.size() != fixed_step_override_left_norms.size() ||
        fixed_step_override_modes.size() != fixed_step_override_right_norms.size())
      {
        throw std::runtime_error(
          "fixed_step_override_modes, fixed_step_override_left_norms and "
          "fixed_step_override_right_norms must have the same length.");
      }
      for (std::size_t idx = 0; idx < fixed_step_override_modes.size(); ++idx) {
        fixed_step_overrides_.push_back(FixedStepOverrideConfig{
          static_cast<int>(fixed_step_override_modes[idx]),
          std::clamp(fixed_step_override_left_norms[idx], -1.0, 1.0),
          std::clamp(fixed_step_override_right_norms[idx], -1.0, 1.0)});
      }
    } else {
      fixed_step_overrides_.push_back(FixedStepOverrideConfig{
        fixed_step_override_mode_,
        fixed_step_override_left_norm_,
        fixed_step_override_right_norm_});
    }

    std::ostringstream stream;
    stream << "Fixed step overrides enabled:";
    for (const auto & fixed_override : fixed_step_overrides_) {
      stream << " [mode=" << fixed_override.mode
             << ", left_norm=" << fixed_override.left_norm
             << ", right_norm=" << fixed_override.right_norm << "]";
    }
    RCLCPP_INFO(this->get_logger(), "%s", stream.str().c_str());
  }

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CmdVelToSerialNode::cmdVelCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_,
    // 2026-05-03 00:12 CST: use volatile durability so /goal_pose from
    // yolo_relative_nav, RViz, and Nav2 tooling all match. Runtime path
    // correction only needs future goals, not replayed latched goals.
    rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&CmdVelToSerialNode::goalCallback, this, std::placeholders::_1));
  path_reference_start_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    path_reference_start_topic_,
    rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&CmdVelToSerialNode::pathReferenceStartCallback, this, std::placeholders::_1));
  path_reference_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    path_reference_topic_,
    rclcpp::QoS(1).reliable().durability_volatile(),
    std::bind(&CmdVelToSerialNode::pathReferenceCallback, this, std::placeholders::_1));
  step_override_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    step_override_topic_,
    rclcpp::QoS(10),
    std::bind(&CmdVelToSerialNode::stepOverrideCallback, this, std::placeholders::_1));
  step_once_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    step_once_topic_,
    rclcpp::QoS(10),
    std::bind(&CmdVelToSerialNode::stepOnceCallback, this, std::placeholders::_1));
  uphill_step_floor_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    uphill_step_floor_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CmdVelToSerialNode::uphillStepFloorStateCallback, this, std::placeholders::_1));
  fixed_step_override_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    fixed_step_override_state_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CmdVelToSerialNode::fixedStepOverrideStateCallback, this, std::placeholders::_1));
  path_correction_profile_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    path_correction_profile_topic_,
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&CmdVelToSerialNode::pathCorrectionProfileCallback, this, std::placeholders::_1));

  mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    mode_topic_,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CmdVelToSerialNode::modeCallback, this, std::placeholders::_1));
  if (
	    odom_y_correction_enabled_ ||
	    path_lateral_correction_enabled_ ||
	    path_yaw_correction_enabled_ ||
	    crouch_yaw_correction_enabled_ ||
	    uphill_pitch_step_floor_enabled_)
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CmdVelToSerialNode::odometryCallback, this, std::placeholders::_1));
  }
  mode_echo_pub_ = this->create_publisher<std_msgs::msg::Int32>(
    mode_echo_topic_, rclcpp::SystemDefaultsQoS());
  serial_connected_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    serial_connected_topic_, rclcpp::SystemDefaultsQoS());
  if (start_signal_rx_enabled_) {
    start_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      start_signal_topic_, rclcpp::QoS(1).reliable().transient_local());
    finish_signal_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      finish_signal_topic_, rclcpp::QoS(10).reliable().durability_volatile());
    action_ack_pub_ = this->create_publisher<std_msgs::msg::String>(
      action_ack_topic_, rclcpp::QoS(10).reliable().durability_volatile());
  }

  const auto transmit_period = std::chrono::duration<double>(1.0 / transmit_rate_hz_);
  transmit_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(transmit_period),
    std::bind(&CmdVelToSerialNode::transmitStepCommand, this));

  publishModeEcho();
  publishSerialConnected(false);
  if (start_signal_rx_enabled_) {
    std_msgs::msg::Bool start_msg;
    start_msg.data = false;
    start_signal_pub_->publish(start_msg);
    RCLCPP_INFO(
      this->get_logger(),
      "serial start signal RX enabled. token=[%s] topic=%s case_sensitive=%s",
      start_signal_token_.c_str(),
      start_signal_topic_.c_str(),
      start_signal_case_sensitive_ ? "true" : "false");
    RCLCPP_INFO(
      this->get_logger(),
      "serial finish signal RX enabled. token=[%s] topic=%s case_sensitive=%s",
      finish_signal_token_.c_str(),
      finish_signal_topic_.c_str(),
      start_signal_case_sensitive_ ? "true" : "false");
    RCLCPP_INFO(
      this->get_logger(),
      "serial action ack RX enabled. jumping_token=[%s] topic=%s",
      jumping_signal_token_.c_str(),
      action_ack_topic_.c_str());
  }

  RCLCPP_INFO(
    this->get_logger(),
    "%scmd_vel->serial bridge started. device=%s baud=%d step_frequency=%.3f Hz default_mode=%d mode_topic=%s mode_echo_topic=%s odom_y_correction=%s path_lateral_correction=%s path_yaw_correction=%s crouch_yaw_correction=%s pose_yaw_offset=%.1fdeg%s",
    styleOrEmpty(kAnsiCyan),
    serial_device_.c_str(),
    baud_rate_,
    mapping_parameters_.step_frequency_hz,
    mode_,
    mode_topic_.c_str(),
    mode_echo_topic_.c_str(),
    odom_y_correction_enabled_ ? "enabled" : "disabled",
    path_lateral_correction_enabled_ ? "enabled" : "disabled",
    path_yaw_correction_enabled_ ? "enabled" : "disabled",
    crouch_yaw_correction_enabled_ ? "enabled" : "disabled",
    pose_yaw_offset_rad_ * 180.0 / kPi,
    resetOrEmpty());
  if (odom_y_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "odom.y step correction enabled. odom_topic=%s goal_topic=%s modes=%zu use_goal_y=%s deadband=%.3f m gain=%.3f norm/m max=%.3f stale_timeout=%.2f s",
      odom_topic_.c_str(),
      goal_topic_.c_str(),
      odom_y_correction_modes_.size(),
      odom_y_correction_use_goal_y_ ? "true" : "false",
      odom_y_correction_deadband_m_,
      odom_y_correction_gain_norm_per_m_,
      odom_y_correction_max_norm_,
      odom_y_correction_stale_timeout_sec_);
  }
  if (path_lateral_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "path lateral step correction enabled. odom_topic=%s goal_topic=%s path_reference_topic=%s modes=%zu deadband=%.3f m gain=%.3f norm/m max=%.3f sign=%.0f stale_timeout=%.2f s",
      odom_topic_.c_str(),
      goal_topic_.c_str(),
      path_reference_topic_.c_str(),
      path_lateral_correction_modes_.size(),
      path_lateral_correction_deadband_m_,
      path_lateral_correction_gain_norm_per_m_,
      path_lateral_correction_max_norm_,
      path_lateral_correction_sign_,
      path_lateral_correction_stale_timeout_sec_);
    RCLCPP_INFO(
      this->get_logger(),
      "path reference explicit start enabled. topic=%s timeout=%.2f s",
      path_reference_start_topic_.c_str(),
      path_reference_start_timeout_sec_);
  }
  if (uphill_path_lateral_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "uphill path lateral correction override enabled. deadband=%.3f m gain=%.3f norm/m max=%.3f sign=%.0f max_wz=%.3f rad/s",
      uphill_path_lateral_correction_deadband_m_,
      uphill_path_lateral_correction_gain_norm_per_m_,
      uphill_path_lateral_correction_max_norm_,
      uphill_path_lateral_correction_sign_,
      uphill_path_lateral_correction_max_abs_wz_rps_);
  }
  if (path_yaw_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "path yaw correction enabled. odom_topic=%s goal_topic=%s path_reference_topic=%s modes=%zu deadband=%.2f deg gain=%.3f norm/rad max=%.3f stale_timeout=%.2f s profile_topic=%s default_profile=%d new_goal_delay=%.2f s",
      odom_topic_.c_str(),
      goal_topic_.c_str(),
      path_reference_topic_.c_str(),
      path_yaw_correction_modes_.size(),
      path_yaw_correction_deadband_rad_ * 180.0 / kPi,
      path_yaw_correction_gain_norm_per_rad_,
      path_yaw_correction_max_norm_,
      path_yaw_correction_stale_timeout_sec_,
      path_correction_profile_topic_.c_str(),
      path_correction_profile_,
      path_correction_new_goal_delay_sec_);
  }
  if (uphill_path_yaw_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "uphill path yaw correction override enabled. modes=%zu deadband=%.2f deg gain=%.3f norm/rad max=%.3f max_wz=%.3f rad/s",
      uphill_path_yaw_correction_modes_.size(),
      uphill_path_yaw_correction_deadband_rad_ * 180.0 / kPi,
      uphill_path_yaw_correction_gain_norm_per_rad_,
      uphill_path_yaw_correction_max_norm_,
      uphill_path_yaw_correction_max_abs_wz_rps_);
  }
  if (crouch_yaw_correction_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "crouch yaw correction enabled. odom_topic=%s goal_topic=%s path_reference_topic=%s modes=%zu deadband=%.2f deg gain=%.3f norm/rad max=%.3f stale_timeout=%.2f s",
      odom_topic_.c_str(),
      goal_topic_.c_str(),
      path_reference_topic_.c_str(),
      crouch_yaw_correction_modes_.size(),
      crouch_yaw_correction_deadband_rad_ * 180.0 / kPi,
      crouch_yaw_correction_gain_norm_per_rad_,
      crouch_yaw_correction_max_norm_,
      crouch_yaw_correction_stale_timeout_sec_);
    if (crouch_yaw_initial_boost_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "crouch yaw initial boost enabled. duration=%.2f s progress=%.3f m min_error=%.2f deg max=%.3f",
        crouch_yaw_initial_boost_duration_sec_,
        crouch_yaw_initial_boost_progress_m_,
        crouch_yaw_initial_boost_min_error_rad_ * 180.0 / kPi,
        crouch_yaw_initial_boost_max_norm_);
    }
    if (crouch_yaw_lateral_priority_enabled_) {
      RCLCPP_INFO(
        this->get_logger(),
        "crouch yaw lateral priority enabled. path_lat_threshold=%.3f m yaw_max_when_active=%.3f",
        crouch_yaw_lateral_priority_error_threshold_m_,
        crouch_yaw_lateral_priority_max_norm_);
    }
  }
  if (same_direction_turn_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "same-direction turn enabled. modes=%zu min_wz=%.3f rad/s min_norm=%.3f",
      same_direction_turn_modes_.size(),
      same_direction_turn_min_wz_rps_,
      same_direction_turn_min_norm_);
  }
  if (step_override_small_opposite_sign_shape_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "step override small opposite-sign shaping enabled. modes=%zu small_norm=%.3f large_norm=%.3f",
      step_override_small_opposite_sign_shape_modes_.size(),
      step_override_small_opposite_sign_shape_small_norm_,
      step_override_small_opposite_sign_shape_large_norm_);
  }
  if (cmd_vel_small_opposite_sign_shape_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "cmd_vel small opposite-sign shaping enabled. modes=%zu small_norm=%.3f large_norm=%.3f",
      cmd_vel_small_opposite_sign_shape_modes_.size(),
      cmd_vel_small_opposite_sign_shape_small_norm_,
      cmd_vel_small_opposite_sign_shape_large_norm_);
  }
  if (min_output_step_norm_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "minimum output step norm enabled. modes=%zu min_norm=%.3f override_modes=%zu",
      min_output_step_norm_modes_.size(),
      min_output_step_norm_,
      min_output_step_norm_override_modes_.size());
  }
  if (uphill_segment_step_floor_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "uphill segment step floor enabled. modes=%zu min_norm=%.3f goal_distance_min=%.2f m max_wz=%.3f forward_only=%s state_topic=%s",
      uphill_segment_step_floor_modes_.size(),
      uphill_segment_step_floor_min_norm_,
      uphill_segment_step_floor_goal_distance_min_m_,
      uphill_segment_step_floor_max_abs_wz_rps_,
      uphill_segment_step_floor_forward_only_ ? "true" : "false",
      uphill_step_floor_state_topic_.c_str());
  }
  if (uphill_pitch_step_floor_enabled_) {
    RCLCPP_INFO(
      this->get_logger(),
      "uphill roll step floor enabled. modes=%zu min_roll=%.2f deg use_abs_roll=%s min_norm=%.3f max_wz=%.3f require_segment=%s state_topic=%s",
      uphill_pitch_step_floor_modes_.size(),
      uphill_pitch_step_floor_min_pitch_rad_ * 180.0 / kPi,
      uphill_pitch_step_floor_use_abs_pitch_ ? "true" : "false",
      uphill_pitch_step_floor_min_norm_,
      uphill_pitch_step_floor_max_abs_wz_rps_,
      uphill_pitch_step_floor_require_segment_enable_ ? "true" : "false",
      uphill_step_floor_state_topic_.c_str());
  }
  RCLCPP_INFO(
    this->get_logger(),
    "fixed step override runtime state topic=%s initial=%s",
    fixed_step_override_state_topic_.c_str(),
    fixed_step_override_runtime_enabled_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(),
    "direct step override enabled. topic=%s timeout=%.2f s one_shot_topic=%s",
    step_override_topic_.c_str(),
    step_override_timeout_sec_,
    step_once_topic_.c_str());
}

CmdVelToSerialNode::~CmdVelToSerialNode()
{
  closeSerialPort();
}

void CmdVelToSerialNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  last_cmd_stamp_ = this->get_clock()->now();
  received_cmd_ = true;
}

void CmdVelToSerialNode::modeCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  const bool changed = msg->data != mode_;
  mode_ = msg->data;
  publishModeEcho();
  if (changed) {
    RCLCPP_INFO(
      this->get_logger(),
      "%sUpdated serial mode to %d%s",
      styleOrEmpty(kAnsiMagenta),
      mode_,
      resetOrEmpty());
  }
}

void CmdVelToSerialNode::uphillStepFloorStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == uphill_step_floor_segment_enabled_) {
    return;
  }
  uphill_step_floor_segment_enabled_ = msg->data;
  RCLCPP_INFO(
    this->get_logger(),
    "%sUpdated uphill step floor segment state to %s%s",
    styleOrEmpty(kAnsiMagenta),
    uphill_step_floor_segment_enabled_ ? "true" : "false",
    resetOrEmpty());
}

void CmdVelToSerialNode::pathCorrectionProfileCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  const int new_profile = std::clamp(msg->data, 0, 3);
  if (new_profile == path_correction_profile_) {
    return;
  }
  path_correction_profile_ = new_profile;
  RCLCPP_INFO(
    this->get_logger(),
    "%sUpdated path correction profile to %d (0=off, 1=normal, 2=uphill_strong, 3=uphill_strong_sign_test)%s",
    styleOrEmpty(kAnsiMagenta),
    path_correction_profile_,
    resetOrEmpty());
}

void CmdVelToSerialNode::fixedStepOverrideStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == fixed_step_override_runtime_enabled_) {
    return;
  }
  fixed_step_override_runtime_enabled_ = msg->data;
  RCLCPP_INFO(
    this->get_logger(),
    "%sUpdated fixed step override runtime state to %s%s",
    styleOrEmpty(kAnsiMagenta),
    fixed_step_override_runtime_enabled_ ? "true" : "false",
    resetOrEmpty());
}

void CmdVelToSerialNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  latest_odom_x_ = msg->pose.pose.position.x;
  latest_odom_y_ = msg->pose.pose.position.y;
  latest_odom_roll_ = rollFromQuaternion(msg->pose.pose.orientation);
  latest_odom_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
  last_odom_stamp_ = this->get_clock()->now();
  received_odom_ = true;
  if (received_goal_ && !path_lateral_start_captured_) {
    path_lateral_start_x_ = latest_odom_x_;
    path_lateral_start_y_ = latest_odom_y_;
    path_lateral_start_captured_ = true;
  }
}

void CmdVelToSerialNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  updatePathReference(*msg, false);
}

void CmdVelToSerialNode::pathReferenceStartCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  pending_path_reference_start_x_ = msg->pose.position.x;
  pending_path_reference_start_y_ = msg->pose.position.y;
  pending_path_reference_start_stamp_ = this->get_clock()->now();
  pending_path_reference_start_received_ = true;
}

void CmdVelToSerialNode::pathReferenceCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  updatePathReference(*msg, true);
}

void CmdVelToSerialNode::updatePathReference(
  const geometry_msgs::msg::PoseStamped & msg,
  bool use_pending_start)
{
  latest_goal_x_ = msg.pose.position.x;
  latest_goal_y_ = msg.pose.position.y;
  latest_goal_yaw_ = yawFromQuaternion(msg.pose.orientation);
  received_goal_ = true;
  last_goal_stamp_ = this->get_clock()->now();
  const bool pending_start_fresh =
    pending_path_reference_start_received_ &&
    (path_reference_start_timeout_sec_ <= 0.0 ||
    (last_goal_stamp_ - pending_path_reference_start_stamp_).seconds() <=
    path_reference_start_timeout_sec_);
  if (use_pending_start && pending_start_fresh) {
    path_lateral_start_x_ = pending_path_reference_start_x_;
    path_lateral_start_y_ = pending_path_reference_start_y_;
    path_lateral_start_captured_ = true;
    pending_path_reference_start_received_ = false;
    return;
  }
  path_lateral_start_x_ = latest_odom_x_;
  path_lateral_start_y_ = latest_odom_y_;
  path_lateral_start_captured_ = received_odom_;
}

void CmdVelToSerialNode::stepOverrideCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) {
    RCLCPP_WARN(
      this->get_logger(),
      "Ignoring step override: expected [mode,left_norm,right_norm], got %zu values",
      msg->data.size());
    return;
  }

  const int override_mode = static_cast<int>(std::lround(msg->data[0]));
  if (override_mode < 0) {
    received_step_override_ = false;
    return;
  }

  step_override_mode_ = override_mode;
  step_override_left_norm_ = std::clamp(static_cast<double>(msg->data[1]), -1.0, 1.0);
  step_override_right_norm_ = std::clamp(static_cast<double>(msg->data[2]), -1.0, 1.0);
  last_step_override_stamp_ = this->get_clock()->now();
  received_step_override_ = true;
}

void CmdVelToSerialNode::stepOnceCallback(
  const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 3) {
    RCLCPP_WARN(
      this->get_logger(),
      "Ignoring one-shot step command: expected [mode,left_norm,right_norm], got %zu values",
      msg->data.size());
    return;
  }

  const int serial_mode = static_cast<int>(std::lround(msg->data[0]));
  if (serial_mode < 0) {
    return;
  }

  const double left_norm = std::clamp(static_cast<double>(msg->data[1]), -1.0, 1.0);
  const double right_norm = std::clamp(static_cast<double>(msg->data[2]), -1.0, 1.0);
  const auto now = this->get_clock()->now();
  if (!ensureSerialOpen(now)) {
    return;
  }

  const auto message = formatNormalizedSerialMessage(serial_mode, left_norm, right_norm);
  // One-shot commands already emit an explicit "serial tx once" log below.
  // Suppress the generic "serial tx" line here so a single write is not
  // mistaken for two serial transmissions in field logs.
  if (!writeSerialMessage(message, now, false)) {
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "%sserial tx once: %s%s",
    styleOrEmpty(kAnsiSerial),
    message.c_str(),
    resetOrEmpty());
}

void CmdVelToSerialNode::transmitStepCommand()
{
  const auto now = this->get_clock()->now();
  auto timed_step_command = computeTimedStepCommand(now);

  if (!ensureSerialOpen(now)) {
    return;
  }

  readSerialInput(now);

  if (timed_step_command.timed_out && !send_zero_on_timeout_) {
    return;
  }

  applyOutputStepSlewLimit(timed_step_command);

  const auto message =
    formatSerialMessage(timed_step_command.step_command, timed_step_command.serial_mode);
  if (!writeSerialMessage(message, now, log_serial_messages_)) {
    return;
  }

  const double left_norm = normalizeStepLength(timed_step_command.step_command.left_step_length_m);
  const double right_norm = normalizeStepLength(timed_step_command.step_command.right_step_length_m);

  if (log_step_debug_ && shouldLogStepDebug(timed_step_command, left_norm, right_norm, now)) {
    const char * step_debug_style = styleOrEmpty(kAnsiBlue);
    if (timed_step_command.timed_out) {
      step_debug_style = styleOrEmpty(kAnsiYellow);
    } else if (timed_step_command.step_override_applied) {
      step_debug_style = styleOrEmpty(kAnsiMagenta);
    } else if (
      timed_step_command.path_lateral_correction_applied ||
      timed_step_command.path_yaw_correction_applied ||
      timed_step_command.odom_y_correction_applied)
    {
      step_debug_style = styleOrEmpty(kAnsiCyan);
    } else if (
      timed_step_command.uphill_segment_step_floor_active ||
      timed_step_command.uphill_pitch_step_floor_active ||
      timed_step_command.min_output_step_norm_applied)
    {
      step_debug_style = styleOrEmpty(kAnsiGreen);
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%sstep_debug mode=%d vx=%.3f wz=%.3f left_m=%.4f right_m=%.4f fwd_m=%.4f step_yaw=%.4f left_norm=%.3f right_norm=%.3f odom_y=%.3f odom_roll=%.1fdeg goal_y_err=%.3f odom_corr=%.3f odom_corr_applied=%s path_lat_err=%.3f path_corr=%.3f path_corr_applied=%s path_yaw_err=%.1fdeg path_yaw_corr=%.3f path_yaw_applied=%s path_profile=%d crouch_yaw_err=%.1fdeg crouch_yaw_corr=%.3f crouch_yaw_applied=%s crouch_fwd_bias=%.3f crouch_fwd_applied=%s same_dir_turn=%s opp_turn_shape=%s uphill_segment=%s uphill_seg_floor=%s uphill_roll_floor=%s min_step_floor=%s override=%s timeout=%s serial=%s%s",
      step_debug_style,
      timed_step_command.serial_mode,
      timed_step_command.vx,
      timed_step_command.wz,
      timed_step_command.step_command.left_step_length_m,
      timed_step_command.step_command.right_step_length_m,
      timed_step_command.step_command.forward_step_length_m,
      timed_step_command.step_command.step_yaw_rad,
      left_norm,
      right_norm,
      timed_step_command.odom_y,
      timed_step_command.odom_roll_rad * 180.0 / kPi,
      timed_step_command.goal_y_error,
      timed_step_command.odom_y_correction_norm,
      timed_step_command.odom_y_correction_applied ? "true" : "false",
      timed_step_command.path_lateral_error,
      timed_step_command.path_lateral_correction_norm,
      timed_step_command.path_lateral_correction_applied ? "true" : "false",
      timed_step_command.path_yaw_error_rad * 180.0 / kPi,
      timed_step_command.path_yaw_correction_norm,
      timed_step_command.path_yaw_correction_applied ? "true" : "false",
      timed_step_command.path_correction_profile,
      timed_step_command.crouch_yaw_error_rad * 180.0 / kPi,
      timed_step_command.crouch_yaw_correction_norm,
      timed_step_command.crouch_yaw_correction_applied ? "true" : "false",
      timed_step_command.crouch_forward_bias_norm,
      timed_step_command.crouch_forward_bias_applied ? "true" : "false",
      timed_step_command.same_direction_turn_applied ? "true" : "false",
      timed_step_command.opposite_sign_turn_shape_applied ? "true" : "false",
      timed_step_command.uphill_step_floor_segment_enabled ? "true" : "false",
      timed_step_command.uphill_segment_step_floor_active ? "true" : "false",
      timed_step_command.uphill_pitch_step_floor_active ? "true" : "false",
      timed_step_command.min_output_step_norm_applied ? "true" : "false",
      timed_step_command.step_override_applied ? "true" : "false",
      timed_step_command.timed_out ? "true" : "false",
      message.c_str(),
      resetOrEmpty());
    updateStepLogCache(timed_step_command, left_norm, right_norm, now);
  }
}

void CmdVelToSerialNode::readSerialInput(const rclcpp::Time & now)
{
  if (!start_signal_rx_enabled_ || serial_fd_ < 0) {
    return;
  }

  int available = 0;
  if (::ioctl(serial_fd_, FIONREAD, &available) != 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      2000,
      "Failed checking serial RX bytes on %s: %s",
      serial_device_.c_str(),
      std::strerror(errno));
    return;
  }

  while (available > 0) {
    char buffer[256];
    const std::size_t requested = std::min<std::size_t>(
      sizeof(buffer),
      static_cast<std::size_t>(available));
    const auto bytes_read = ::read(serial_fd_, buffer, requested);
    if (bytes_read < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "Failed reading from %s: %s",
        serial_device_.c_str(),
        std::strerror(errno));
      closeSerialPort();
      return;
    }
    if (bytes_read == 0) {
      return;
    }

    handleSerialRxBytes(buffer, static_cast<std::size_t>(bytes_read), now);

    if (::ioctl(serial_fd_, FIONREAD, &available) != 0) {
      return;
    }
  }
}

void CmdVelToSerialNode::handleSerialRxBytes(
  const char * data,
  std::size_t size,
  const rclcpp::Time & now)
{
  for (std::size_t idx = 0; idx < size; ++idx) {
    const char ch = data[idx];
    if (ch == '[') {
      serial_rx_frame_buffer_.clear();
      serial_rx_frame_buffer_.push_back(ch);
      serial_rx_in_frame_ = true;
      continue;
    }

    if (!serial_rx_in_frame_) {
      continue;
    }

    serial_rx_frame_buffer_.push_back(ch);
    if (serial_rx_frame_buffer_.size() > serial_rx_max_frame_len_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Dropping overlong serial RX frame, max_len=%zu",
        serial_rx_max_frame_len_);
      serial_rx_frame_buffer_.clear();
      serial_rx_in_frame_ = false;
      continue;
    }

    if (ch == ']') {
      handleSerialRxFrame(serial_rx_frame_buffer_, now);
      serial_rx_frame_buffer_.clear();
      serial_rx_in_frame_ = false;
    }
  }
}

void CmdVelToSerialNode::handleSerialRxFrame(
  const std::string & frame,
  const rclcpp::Time & now)
{
  if (frame.size() < 2 || frame.front() != '[' || frame.back() != ']') {
    return;
  }

  const auto payload = trimAscii(frame.substr(1, frame.size() - 2));
  if (log_serial_rx_frames_) {
    RCLCPP_INFO(
      this->get_logger(),
      "%sserial rx: [%s]%s",
      styleOrEmpty(kAnsiSerial),
      payload.c_str(),
      resetOrEmpty());
  }

  // The controller may append a route mode, for example [start,0] or [start,1].
  // Match the command field here so later start frames still work as retry signals.
  const auto command_separator = payload.find(',');
  const auto command = trimAscii(payload.substr(0, command_separator));
  const auto expected_start = trimAscii(start_signal_token_);
  const bool matched_start =
    start_signal_case_sensitive_
      ? command == expected_start
      : toLowerAscii(command) == toLowerAscii(expected_start);
  if (matched_start) {
    publishStartSignal(now);
    return;
  }

  const auto expected_finish = trimAscii(finish_signal_token_);
  const bool matched_finish =
    start_signal_case_sensitive_
      ? payload == expected_finish
      : toLowerAscii(payload) == toLowerAscii(expected_finish);
  if (matched_finish) {
    publishActionAck(payload, now);
    publishFinishSignal(now);
    return;
  }

  const auto expected_jumping = trimAscii(jumping_signal_token_);
  const bool matched_jumping =
    start_signal_case_sensitive_
      ? payload == expected_jumping
      : toLowerAscii(payload) == toLowerAscii(expected_jumping);
  if (matched_jumping) {
    publishActionAck(payload, now);
  }
}

void CmdVelToSerialNode::publishStartSignal(const rclcpp::Time &)
{
  if (!start_signal_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  start_signal_pub_->publish(msg);

  if (!start_signal_received_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received serial start token [%s], published %s=true",
      start_signal_token_.c_str(),
      start_signal_topic_.c_str());
  }
  start_signal_received_ = true;
}

void CmdVelToSerialNode::publishFinishSignal(const rclcpp::Time &)
{
  if (!finish_signal_pub_) {
    return;
  }

  std_msgs::msg::Bool msg;
  msg.data = true;
  finish_signal_pub_->publish(msg);

  if (!finish_signal_received_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Received serial finish token [%s], published %s=true",
      finish_signal_token_.c_str(),
      finish_signal_topic_.c_str());
  }
  finish_signal_received_ = true;
}

void CmdVelToSerialNode::publishActionAck(const std::string & token, const rclcpp::Time &)
{
  if (!action_ack_pub_) {
    return;
  }

  std_msgs::msg::String msg;
  msg.data = trimAscii(token);
  action_ack_pub_->publish(msg);

  RCLCPP_INFO(
    this->get_logger(),
    "Received serial action ack [%s], published %s",
    msg.data.c_str(),
    action_ack_topic_.c_str());
}

CmdVelToSerialNode::TimedStepCommand
CmdVelToSerialNode::computeTimedStepCommand(const rclcpp::Time & stamp) const
{
  TimedStepCommand command;
  command.serial_mode = mode_;
  command.odom_roll_rad = latest_odom_roll_;
  command.uphill_step_floor_segment_enabled = uphill_step_floor_segment_enabled_;
  command.path_correction_profile = path_correction_profile_;

  const bool step_override_active =
    received_step_override_ &&
    (step_override_timeout_sec_ <= 0.0 ||
    (stamp - last_step_override_stamp_).seconds() <= step_override_timeout_sec_);
  if (step_override_active) {
    const double max_step_length_m = mapping_parameters_.max_step_length_m;
    const double left_step_length_m = step_override_left_norm_ * max_step_length_m;
    const double right_step_length_m = step_override_right_norm_ * max_step_length_m;
    command.serial_mode = step_override_mode_;
    command.step_command.left_step_length_m = left_step_length_m;
    command.step_command.right_step_length_m = right_step_length_m;
    command.step_command.forward_step_length_m = 0.5 * (left_step_length_m + right_step_length_m);
    command.step_command.step_yaw_rad =
      (right_step_length_m - left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    command.step_override_applied = true;
    // Runtime overrides are still eligible for mode-scoped corrections.
    // This lets the down-stair mode=1 direct step keep its full base stride
    // while applying small left/right trims from path/lateral yaw correction.
    // Stop overrides remain zero because correction functions reject zero
    // forward step, and mode=6 spin is not in the path correction mode lists.
    applyOdomYCorrection(command, stamp);
    applyPathLateralCorrection(command, stamp);
    applyPathYawCorrection(command, stamp);
    applyCrouchYawCorrection(command, stamp);
    applyCrouchForwardBias(command);
    applySameDirectionTurnBias(command);
    applyMinOutputStepNorm(command, stamp);
    applyOppositeSignTurnShape(command);
    return command;
  }

  const auto cmd_age_sec = (stamp - last_cmd_stamp_).seconds();
  const bool stale = !received_cmd_ || cmd_age_sec > cmd_vel_timeout_sec_;

  double vx = latest_cmd_vel_.linear.x;
  double wz = latest_cmd_vel_.angular.z;

  if (stale) {
    command.timed_out = true;
    vx = 0.0;
    wz = 0.0;
  }

  command.vx = vx;
  command.wz = wz;

  // 2026-05-03 00:23 CST: keep YAML fixed-step mappings available for normal
  // obstacle_manager use, but allow yolo_relative_nav to temporarily disable
  // them while it commands route_mode=6 through nav_executor. Rollback: remove
  // fixed_step_override_runtime_enabled_ from this condition if runtime gating
  // is no longer needed.
  const bool fixed_step_override_active =
    !command.timed_out && fixed_step_override_enabled_ && fixed_step_override_runtime_enabled_;
  const auto * fixed_step_override =
    fixed_step_override_active ? findFixedStepOverride(mode_) : nullptr;
  if (fixed_step_override != nullptr) {
    const double max_step_length_m = mapping_parameters_.max_step_length_m;
    const double left_step_length_m = fixed_step_override->left_norm * max_step_length_m;
    const double right_step_length_m = fixed_step_override->right_norm * max_step_length_m;
    command.step_command.left_step_length_m = left_step_length_m;
    command.step_command.right_step_length_m = right_step_length_m;
    command.step_command.forward_step_length_m = 0.5 * (left_step_length_m + right_step_length_m);
    command.step_command.step_yaw_rad =
      (right_step_length_m - left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    applyOdomYCorrection(command, stamp);
    applyPathLateralCorrection(command, stamp);
    // 2026-05-01: mode=0 may drift after turns/jumps; path-heading yaw correction is segment-scoped.
    applyPathYawCorrection(command, stamp);
    // 2026-04-30: platform spin can leave crouch gait yaw-biased; correct mode-scoped posture before final filters.
    applyCrouchYawCorrection(command, stamp);
    applyCrouchForwardBias(command);
    applySameDirectionTurnBias(command);
    applyMinOutputStepNorm(command, stamp);
    applyOppositeSignTurnShape(command);
    return command;
  }

  command.step_command = mapper_->compute(vx, wz);
  applyOdomYCorrection(command, stamp);
  applyPathLateralCorrection(command, stamp);
  // 2026-05-01: use current goal segment heading for normal walking posture correction.
  applyPathYawCorrection(command, stamp);
  // 2026-04-30: keep crouch walking locked to the waypoint yaw after platform spin.
  applyCrouchYawCorrection(command, stamp);
  applyCrouchForwardBias(command);
  applySameDirectionTurnBias(command);
  applyMinOutputStepNorm(command, stamp);
  applyOppositeSignTurnShape(command);
  return command;

}

void CmdVelToSerialNode::applyOdomYCorrection(
  TimedStepCommand & command,
  const rclcpp::Time & stamp) const
{
  command.odom_y = latest_odom_y_;
  command.goal_y = latest_goal_y_;

  if (!odom_y_correction_enabled_ || !received_odom_) {
    return;
  }

  if (!modeInList(mode_, odom_y_correction_modes_)) {
    return;
  }

  if (
    odom_y_correction_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > odom_y_correction_stale_timeout_sec_)
  {
    return;
  }

  if (std::abs(command.step_command.forward_step_length_m) <= 1e-6) {
    return;
  }

  if (std::abs(command.wz) > odom_y_correction_max_abs_wz_rps_) {
    return;
  }

  double correction_sign = 0.0;
  double excess_y = 0.0;
  if (odom_y_correction_use_goal_y_ && received_goal_) {
    const double goal_distance_m = std::hypot(
      latest_goal_x_ - latest_odom_x_,
      latest_goal_y_ - latest_odom_y_);
    if (goal_distance_m < odom_y_correction_goal_distance_min_m_) {
      return;
    }
    command.goal_y_error = latest_odom_y_ - latest_goal_y_;
    if (std::abs(command.goal_y_error) <= odom_y_correction_deadband_m_) {
      return;
    }
    correction_sign = command.goal_y_error > 0.0 ? 1.0 : -1.0;
    excess_y = std::abs(command.goal_y_error) - odom_y_correction_deadband_m_;
  } else if (odom_y_correction_positive_only_) {
    if (latest_odom_y_ <= odom_y_correction_deadband_m_) {
      return;
    }
    command.goal_y_error = latest_odom_y_;
    correction_sign = 1.0;
    excess_y = latest_odom_y_ - odom_y_correction_deadband_m_;
  } else {
    if (std::abs(latest_odom_y_) <= odom_y_correction_deadband_m_) {
      return;
    }
    command.goal_y_error = latest_odom_y_;
    correction_sign = latest_odom_y_ > 0.0 ? 1.0 : -1.0;
    excess_y = std::abs(latest_odom_y_) - odom_y_correction_deadband_m_;
  }

  const double correction_norm = std::clamp(
    excess_y * odom_y_correction_gain_norm_per_m_,
    0.0,
    odom_y_correction_max_norm_);
  if (correction_norm <= 1e-6) {
    return;
  }

  const double correction_step_m =
    correction_sign * correction_norm * mapping_parameters_.max_step_length_m;
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double corrected_left = std::clamp(
    command.step_command.left_step_length_m + correction_step_m,
    -max_step_length_m,
    max_step_length_m);
  const double corrected_right = std::clamp(
    command.step_command.right_step_length_m - correction_step_m,
    -max_step_length_m,
    max_step_length_m);

  command.step_command.left_step_length_m = corrected_left;
  command.step_command.right_step_length_m = corrected_right;
  command.step_command.forward_step_length_m = 0.5 * (corrected_left + corrected_right);
  command.step_command.step_yaw_rad =
    (corrected_right - corrected_left) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.odom_y_correction_norm = correction_sign * correction_norm;
  command.odom_y_correction_applied = true;
}

void CmdVelToSerialNode::applyPathLateralCorrection(
  TimedStepCommand & command,
  const rclcpp::Time & stamp) const
{
  if (!path_lateral_correction_enabled_ || !received_odom_ || !received_goal_) {
    return;
  }

  // 2026-05-01: serial-side path correction is now waypoint-profile gated.
  // Old behavior was profile=1 everywhere; profile=0 disables it for unstable
  // post-jump bridge segments, profile=2 keeps the stronger uphill tune, and
  // profile=3 currently validates the non-mirrored lateral sign on mirrored runs.
  if (path_correction_profile_ <= 0) {
    return;
  }

  if (
    path_correction_new_goal_delay_sec_ > 0.0 &&
    (stamp - last_goal_stamp_).seconds() < path_correction_new_goal_delay_sec_)
  {
    return;
  }

  if (!path_lateral_start_captured_) {
    return;
  }

  if (!modeInList(command.serial_mode, path_lateral_correction_modes_)) {
    return;
  }

  if (
    path_lateral_correction_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > path_lateral_correction_stale_timeout_sec_)
  {
    return;
  }

  if (std::abs(command.step_command.forward_step_length_m) <= 1e-6) {
    return;
  }

  // Keep the uphill speed floor independent from correction strength. Profile 1
  // means normal correction even when the uphill step-floor segment is latched;
  // profile 2/3 opt into the stronger uphill lateral parameters explicitly.
  const bool use_uphill_lateral_params =
    path_correction_profile_ >= 2 &&
    uphill_path_lateral_correction_enabled_;
  const double deadband_m = use_uphill_lateral_params ?
    uphill_path_lateral_correction_deadband_m_ :
    path_lateral_correction_deadband_m_;
  const double gain_norm_per_m = use_uphill_lateral_params ?
    uphill_path_lateral_correction_gain_norm_per_m_ :
    path_lateral_correction_gain_norm_per_m_;
  const double max_norm = use_uphill_lateral_params ?
    uphill_path_lateral_correction_max_norm_ :
    path_lateral_correction_max_norm_;
  const double goal_distance_min_m = use_uphill_lateral_params ?
    uphill_path_lateral_correction_goal_distance_min_m_ :
    path_lateral_correction_goal_distance_min_m_;
  const double max_abs_wz_rps = use_uphill_lateral_params ?
    uphill_path_lateral_correction_max_abs_wz_rps_ :
    path_lateral_correction_max_abs_wz_rps_;

  if (std::abs(command.wz) > max_abs_wz_rps) {
    return;
  }

  const double seg_x = latest_goal_x_ - path_lateral_start_x_;
  const double seg_y = latest_goal_y_ - path_lateral_start_y_;
  const double seg_len = std::hypot(seg_x, seg_y);
  if (seg_len < path_lateral_correction_min_segment_length_m_) {
    return;
  }

  const double goal_distance_m = std::hypot(
    latest_goal_x_ - latest_odom_x_,
    latest_goal_y_ - latest_odom_y_);
  if (goal_distance_m < goal_distance_min_m) {
    return;
  }

  const double rel_x = latest_odom_x_ - path_lateral_start_x_;
  const double rel_y = latest_odom_y_ - path_lateral_start_y_;
  const double lateral_sign =
    use_uphill_lateral_params ? uphill_path_lateral_correction_sign_ : path_lateral_correction_sign_;
  const double signed_lateral_error =
    lateral_sign * (seg_x * rel_y - seg_y * rel_x) / seg_len;
  command.path_lateral_error = signed_lateral_error;
  if (std::abs(signed_lateral_error) <= deadband_m) {
    return;
  }

  const double excess_lateral =
    std::abs(signed_lateral_error) - deadband_m;
  const double correction_norm = std::clamp(
    excess_lateral * gain_norm_per_m,
    0.0,
    max_norm);
  if (correction_norm <= 1e-6) {
    return;
  }

  const double correction_sign = signed_lateral_error > 0.0 ? 1.0 : -1.0;
  if (modeInList(command.serial_mode, path_lateral_direct_override_modes_)) {
    const double max_step_length_m = mapping_parameters_.max_step_length_m;
    if (signed_lateral_error > 0.0) {
      command.step_command.left_step_length_m = path_lateral_direct_high_norm_ * max_step_length_m;
      command.step_command.right_step_length_m = path_lateral_direct_low_norm_ * max_step_length_m;
    } else {
      command.step_command.left_step_length_m = path_lateral_direct_low_norm_ * max_step_length_m;
      command.step_command.right_step_length_m = path_lateral_direct_high_norm_ * max_step_length_m;
    }
    command.step_command.forward_step_length_m = 0.5 * (
      command.step_command.left_step_length_m + command.step_command.right_step_length_m);
    command.step_command.step_yaw_rad =
      (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    command.path_lateral_correction_norm =
      correction_sign * std::abs(path_lateral_direct_high_norm_ - path_lateral_direct_low_norm_);
    command.path_lateral_correction_applied = true;
    return;
  }

  const double correction_step_m =
    correction_sign * correction_norm * mapping_parameters_.max_step_length_m;
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double corrected_left = std::clamp(
    command.step_command.left_step_length_m + correction_step_m,
    -max_step_length_m,
    max_step_length_m);
  const double corrected_right = std::clamp(
    command.step_command.right_step_length_m - correction_step_m,
    -max_step_length_m,
    max_step_length_m);

  command.step_command.left_step_length_m = corrected_left;
  command.step_command.right_step_length_m = corrected_right;
  command.step_command.forward_step_length_m = 0.5 * (corrected_left + corrected_right);
  command.step_command.step_yaw_rad =
    (corrected_right - corrected_left) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.path_lateral_correction_norm = correction_sign * correction_norm;
  command.path_lateral_correction_applied = true;
}

void CmdVelToSerialNode::applyPathYawCorrection(
  TimedStepCommand & command,
  const rclcpp::Time & stamp) const
{
  if (!path_yaw_correction_enabled_ || !received_odom_ || !received_goal_) {
    return;
  }

  // 2026-05-01: keep normal walking yaw correction out of unstable post-jump
  // transfer segments unless obstacle_manager explicitly enables it for the interval.
  if (path_correction_profile_ <= 0) {
    return;
  }

  if (
    path_correction_new_goal_delay_sec_ > 0.0 &&
    (stamp - last_goal_stamp_).seconds() < path_correction_new_goal_delay_sec_)
  {
    return;
  }

  if (!path_lateral_start_captured_) {
    return;
  }

  if (!modeInList(command.serial_mode, path_yaw_correction_modes_)) {
    return;
  }

  if (
    path_yaw_correction_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > path_yaw_correction_stale_timeout_sec_)
  {
    return;
  }

  if (std::abs(command.step_command.forward_step_length_m) <= 1e-6) {
    return;
  }

  // Keep the uphill speed floor independent from yaw correction strength. Profile
  // 1 should remain the normal yaw loop; profile 2/3 opt into uphill yaw recovery.
  const bool use_uphill_yaw_params =
    path_correction_profile_ >= 2 &&
    uphill_path_yaw_correction_enabled_ &&
    modeInList(command.serial_mode, uphill_path_yaw_correction_modes_);
  const double deadband_rad = use_uphill_yaw_params ?
    uphill_path_yaw_correction_deadband_rad_ :
    path_yaw_correction_deadband_rad_;
  const double gain_norm_per_rad = use_uphill_yaw_params ?
    uphill_path_yaw_correction_gain_norm_per_rad_ :
    path_yaw_correction_gain_norm_per_rad_;
  const double max_norm = use_uphill_yaw_params ?
    uphill_path_yaw_correction_max_norm_ :
    path_yaw_correction_max_norm_;
  const double goal_distance_min_m = use_uphill_yaw_params ?
    uphill_path_yaw_correction_goal_distance_min_m_ :
    path_yaw_correction_goal_distance_min_m_;
  const double max_abs_wz_rps = use_uphill_yaw_params ?
    uphill_path_yaw_correction_max_abs_wz_rps_ :
    path_yaw_correction_max_abs_wz_rps_;

  if (std::abs(command.wz) > max_abs_wz_rps) {
    return;
  }

  const double seg_x = latest_goal_x_ - path_lateral_start_x_;
  const double seg_y = latest_goal_y_ - path_lateral_start_y_;
  const double seg_len = std::hypot(seg_x, seg_y);
  if (seg_len < path_yaw_correction_min_segment_length_m_) {
    return;
  }

  const double goal_distance_m = std::hypot(
    latest_goal_x_ - latest_odom_x_,
    latest_goal_y_ - latest_odom_y_);
  if (goal_distance_m < goal_distance_min_m) {
    return;
  }

  const double desired_yaw = std::atan2(seg_y, seg_x);
  const double yaw_error_rad = normalizeAngle(desired_yaw - controllerYaw());
  command.path_yaw_error_rad = yaw_error_rad;
  const double excess_yaw_rad = std::abs(yaw_error_rad) - deadband_rad;
  if (excess_yaw_rad <= 0.0) {
    return;
  }

  const double correction_sign = yaw_error_rad > 0.0 ? 1.0 : -1.0;
  if (modeInList(command.serial_mode, path_yaw_direct_override_modes_)) {
    const double max_step_length_m = mapping_parameters_.max_step_length_m;
    if (yaw_error_rad < 0.0) {
      command.step_command.left_step_length_m = path_yaw_direct_high_norm_ * max_step_length_m;
      command.step_command.right_step_length_m = path_yaw_direct_low_norm_ * max_step_length_m;
    } else {
      command.step_command.left_step_length_m = path_yaw_direct_low_norm_ * max_step_length_m;
      command.step_command.right_step_length_m = path_yaw_direct_high_norm_ * max_step_length_m;
    }
    command.step_command.forward_step_length_m = 0.5 * (
      command.step_command.left_step_length_m + command.step_command.right_step_length_m);
    command.step_command.step_yaw_rad =
      (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    command.path_yaw_correction_norm =
      correction_sign * std::abs(path_yaw_direct_high_norm_ - path_yaw_direct_low_norm_);
    command.path_yaw_correction_applied = true;
    return;
  }

  const double correction_norm = std::clamp(
    excess_yaw_rad * gain_norm_per_rad,
    0.0,
    max_norm);
  if (correction_norm <= 1e-6) {
    return;
  }

  const double correction_step_m =
    correction_sign * correction_norm * mapping_parameters_.max_step_length_m;
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double corrected_left = std::clamp(
    command.step_command.left_step_length_m - correction_step_m,
    -max_step_length_m,
    max_step_length_m);
  const double corrected_right = std::clamp(
    command.step_command.right_step_length_m + correction_step_m,
    -max_step_length_m,
    max_step_length_m);

  command.step_command.left_step_length_m = corrected_left;
  command.step_command.right_step_length_m = corrected_right;
  command.step_command.forward_step_length_m = 0.5 * (corrected_left + corrected_right);
  command.step_command.step_yaw_rad =
    (corrected_right - corrected_left) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.path_yaw_correction_norm = correction_sign * correction_norm;
  command.path_yaw_correction_applied = true;
}

void CmdVelToSerialNode::applyCrouchYawCorrection(
  TimedStepCommand & command,
  const rclcpp::Time & stamp) const
{
  if (!crouch_yaw_correction_enabled_ || !received_odom_ || !received_goal_) {
    return;
  }

  if (!modeInList(command.serial_mode, crouch_yaw_correction_modes_)) {
    return;
  }

  if (
    crouch_yaw_correction_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > crouch_yaw_correction_stale_timeout_sec_)
  {
    return;
  }

  if (std::abs(command.step_command.forward_step_length_m) <= 1e-6) {
    return;
  }

  if (std::abs(command.wz) > crouch_yaw_correction_max_abs_wz_rps_) {
    return;
  }

  const double goal_distance_m = std::hypot(
    latest_goal_x_ - latest_odom_x_,
    latest_goal_y_ - latest_odom_y_);
  if (goal_distance_m < crouch_yaw_correction_goal_distance_min_m_) {
    return;
  }

  // 2026-05-01 reverted: controllerYaw() made crouch correction diverge in tests.
  // Keep mode=2 crouch walking on the original odom-yaw frame.
  // 2026-05-01 old: const double yaw_error_rad = normalizeAngle(latest_goal_yaw_ - controllerYaw());
  const double yaw_error_rad = normalizeAngle(latest_goal_yaw_ - latest_odom_yaw_);
  command.crouch_yaw_error_rad = yaw_error_rad;
  const double excess_yaw_rad = std::abs(yaw_error_rad) - crouch_yaw_correction_deadband_rad_;
  if (excess_yaw_rad <= 0.0) {
    return;
  }

  double max_crouch_yaw_norm = crouch_yaw_correction_max_norm_;
  if (
    crouch_yaw_initial_boost_enabled_ &&
    crouch_yaw_initial_boost_max_norm_ > max_crouch_yaw_norm &&
    std::abs(yaw_error_rad) >= crouch_yaw_initial_boost_min_error_rad_)
  {
    bool boost_active = false;
    const double elapsed_since_goal_sec = (stamp - last_goal_stamp_).seconds();
    if (
      crouch_yaw_initial_boost_duration_sec_ > 0.0 &&
      elapsed_since_goal_sec >= 0.0 &&
      elapsed_since_goal_sec <= crouch_yaw_initial_boost_duration_sec_)
    {
      boost_active = true;
    }

    if (
      !boost_active &&
      crouch_yaw_initial_boost_progress_m_ > 0.0 &&
      path_lateral_start_captured_)
    {
      const double seg_x = latest_goal_x_ - path_lateral_start_x_;
      const double seg_y = latest_goal_y_ - path_lateral_start_y_;
      const double seg_len = std::hypot(seg_x, seg_y);
      if (seg_len > 1e-6) {
        const double rel_x = latest_odom_x_ - path_lateral_start_x_;
        const double rel_y = latest_odom_y_ - path_lateral_start_y_;
        const double along_track_m = (seg_x * rel_x + seg_y * rel_y) / seg_len;
        if (
          along_track_m >= -0.05 &&
          along_track_m <= crouch_yaw_initial_boost_progress_m_)
        {
          boost_active = true;
        }
      }
    }

    if (boost_active) {
      max_crouch_yaw_norm = crouch_yaw_initial_boost_max_norm_;
    }
  }

  if (
    crouch_yaw_lateral_priority_enabled_ &&
    command.path_lateral_correction_applied &&
    std::abs(command.path_lateral_error) >= crouch_yaw_lateral_priority_error_threshold_m_)
  {
    max_crouch_yaw_norm =
      std::min(max_crouch_yaw_norm, crouch_yaw_lateral_priority_max_norm_);
  }

  const double correction_norm = std::clamp(
    excess_yaw_rad * crouch_yaw_correction_gain_norm_per_rad_,
    0.0,
    max_crouch_yaw_norm);
  if (correction_norm <= 1e-6) {
    return;
  }

  const double correction_sign = yaw_error_rad > 0.0 ? 1.0 : -1.0;
  const double correction_step_m =
    correction_sign * correction_norm * mapping_parameters_.max_step_length_m;
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double corrected_left = std::clamp(
    command.step_command.left_step_length_m - correction_step_m,
    -max_step_length_m,
    max_step_length_m);
  const double corrected_right = std::clamp(
    command.step_command.right_step_length_m + correction_step_m,
    -max_step_length_m,
    max_step_length_m);

  command.step_command.left_step_length_m = corrected_left;
  command.step_command.right_step_length_m = corrected_right;
  command.step_command.forward_step_length_m = 0.5 * (corrected_left + corrected_right);
  command.step_command.step_yaw_rad =
    (corrected_right - corrected_left) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.crouch_yaw_correction_norm = correction_sign * correction_norm;
  command.crouch_yaw_correction_applied = true;
}

void CmdVelToSerialNode::applySameDirectionTurnBias(TimedStepCommand & command) const
{
  if (!same_direction_turn_enabled_) {
    return;
  }
  if (!modeInList(mode_, same_direction_turn_modes_)) {
    return;
  }
  if (command.vx <= mapping_parameters_.min_effective_linear_speed_mps) {
    return;
  }
  if (std::abs(command.wz) < same_direction_turn_min_wz_rps_) {
    return;
  }

  const double min_positive_step_m = same_direction_turn_min_norm_ * mapping_parameters_.max_step_length_m;
  const double min_step_m = std::min(
    command.step_command.left_step_length_m,
    command.step_command.right_step_length_m);
  if (min_step_m >= min_positive_step_m) {
    return;
  }

  const double common_bias_m = min_positive_step_m - min_step_m;
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  command.step_command.left_step_length_m = std::clamp(
    command.step_command.left_step_length_m + common_bias_m,
    0.0,
    max_step_length_m);
  command.step_command.right_step_length_m = std::clamp(
    command.step_command.right_step_length_m + common_bias_m,
    0.0,
    max_step_length_m);
  command.step_command.forward_step_length_m = 0.5 * (
    command.step_command.left_step_length_m + command.step_command.right_step_length_m);
  command.step_command.step_yaw_rad =
    (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.same_direction_turn_applied = true;
}

void CmdVelToSerialNode::applyCrouchForwardBias(TimedStepCommand & command) const
{
  if (!crouch_forward_bias_enabled_) {
    return;
  }
  if (!modeInList(command.serial_mode, crouch_forward_bias_modes_)) {
    return;
  }
  if (command.step_override_applied || command.timed_out) {
    return;
  }
  if (command.vx <= 1e-6) {
    return;
  }
  if (std::abs(command.wz) > crouch_forward_bias_max_abs_wz_rps_) {
    return;
  }

  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double min_forward_step_m = crouch_forward_bias_min_forward_step_norm_ * max_step_length_m;
  if (command.step_command.forward_step_length_m < min_forward_step_m) {
    return;
  }
  if (
    command.step_command.left_step_length_m < -1e-6 ||
    command.step_command.right_step_length_m < -1e-6)
  {
    return;
  }

  const double bias_step_m = crouch_forward_bias_norm_ * max_step_length_m;
  if (bias_step_m <= 1e-6) {
    return;
  }

  const double corrected_left = std::clamp(
    command.step_command.left_step_length_m + bias_step_m,
    0.0,
    max_step_length_m);
  const double corrected_right = std::clamp(
    command.step_command.right_step_length_m + bias_step_m,
    0.0,
    max_step_length_m);

  command.step_command.left_step_length_m = corrected_left;
  command.step_command.right_step_length_m = corrected_right;
  command.step_command.forward_step_length_m = 0.5 * (corrected_left + corrected_right);
  command.step_command.step_yaw_rad =
    (corrected_right - corrected_left) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.crouch_forward_bias_norm = crouch_forward_bias_norm_;
  command.crouch_forward_bias_applied = true;
}

void CmdVelToSerialNode::applyMinOutputStepNorm(
  TimedStepCommand & command,
  const rclcpp::Time & stamp) const
{
  if (!min_output_step_norm_enabled_) {
    return;
  }
  if (command.timed_out) {
    return;
  }

  double min_output_step_norm = 0.0;
  // 2026-05-23:
  // Uphill can stall before roll exceeds the stronger 0.85 floor threshold.
  // Keep a segment-scoped medium floor first, then let roll-triggered uphill
  // floor take over once the robot is visibly on the ramp.
  bool uphill_segment_floor_active = false;
  bool uphill_pitch_floor_active = false;
  if (uphillSegmentStepFloorForCommand(command, stamp, min_output_step_norm)) {
    uphill_segment_floor_active = true;
    command.uphill_segment_step_floor_active = true;
  }
  if (uphillPitchStepFloorForCommand(command, stamp, min_output_step_norm)) {
    uphill_pitch_floor_active = true;
    command.uphill_pitch_step_floor_active = true;
    command.uphill_segment_step_floor_active = false;
  } else if (!uphill_segment_floor_active &&
    !minOutputStepNormForMode(command.serial_mode, min_output_step_norm))
  {
    return;
  }
  if (min_output_step_norm <= 1e-6) {
    return;
  }

  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double min_abs_step_m = min_output_step_norm * max_step_length_m;
  const double original_left_step_m = command.step_command.left_step_length_m;
  const double original_right_step_m = command.step_command.right_step_length_m;
  const double original_left_norm = normalizeStepLength(original_left_step_m);
  const double original_right_norm = normalizeStepLength(original_right_step_m);
  const bool opposite_sign_step_override =
    command.step_override_applied && (original_left_norm * original_right_norm) < -1e-6;
  const bool step_override_small_opposite_sign_shape =
    step_override_small_opposite_sign_shape_enabled_ &&
    command.step_override_applied &&
    modeInList(command.serial_mode, step_override_small_opposite_sign_shape_modes_) &&
    opposite_sign_step_override &&
    std::max(std::abs(original_left_norm), std::abs(original_right_norm)) <=
    (min_output_step_norm + 1e-6);
  const bool cmd_vel_small_opposite_sign_shape =
    cmd_vel_small_opposite_sign_shape_enabled_ &&
    !command.step_override_applied &&
    modeInList(command.serial_mode, cmd_vel_small_opposite_sign_shape_modes_) &&
    (original_left_norm * original_right_norm) < -1e-6 &&
    std::max(std::abs(original_left_norm), std::abs(original_right_norm)) <=
    (min_output_step_norm + 1e-6);
  const bool small_opposite_sign_step_override =
    opposite_sign_step_override &&
    (
      (command.serial_mode == 0 && std::abs(original_left_norm + original_right_norm) <= 0.05) ||
      (
        command.serial_mode == 6 &&
        std::abs(original_left_norm + original_right_norm) <= 0.15 &&
        std::max(std::abs(original_left_norm), std::abs(original_right_norm)) <= 0.30
      )
    );
  if (step_override_small_opposite_sign_shape)
  {
    // Bridge-entry yaw recovery occasionally emits tiny opposite-sign mode=0
    // overrides that the output floor expands into symmetric [-0.20,+0.20].
    // Shape them into an asymmetric pair instead to avoid a hard in-place jerk.
    // For this special mode=0 bridge case, the negative side should always be
    // the smaller-magnitude leg regardless of parameter ordering in YAML.
    const double configured_small_norm = std::clamp(
      step_override_small_opposite_sign_shape_small_norm_,
      0.0,
      1.0);
    const double configured_large_norm = std::clamp(
      step_override_small_opposite_sign_shape_large_norm_,
      0.0,
      1.0);
    const double negative_norm = std::clamp(
      std::min(configured_small_norm, configured_large_norm),
      0.0,
      1.0);
    const double positive_norm = std::clamp(
      std::max(configured_small_norm, configured_large_norm),
      0.0,
      1.0);
    const double negative_step_m = negative_norm * max_step_length_m;
    const double positive_step_m = positive_norm * max_step_length_m;
    const bool ccw_turn = original_left_norm < 0.0 && original_right_norm > 0.0;
    command.step_command.left_step_length_m = ccw_turn ? -negative_step_m : positive_step_m;
    command.step_command.right_step_length_m = ccw_turn ? positive_step_m : -negative_step_m;
    command.step_command.forward_step_length_m = 0.5 * (
      command.step_command.left_step_length_m + command.step_command.right_step_length_m);
    command.step_command.step_yaw_rad =
      (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    command.min_output_step_norm_applied = true;
    command.opposite_sign_turn_shape_applied = true;
    return;
  }
  if (cmd_vel_small_opposite_sign_shape)
  {
    // AprilTag close-range alignment sends small cmd_vel yaw corrections. The
    // mode=0 output floor used to expand those into symmetric [-0.20,+0.20].
    // Keep the positive side at the floor but make the negative side smaller,
    // which softens in-place correction without changing higher-level logic.
    const double configured_small_norm = std::clamp(
      cmd_vel_small_opposite_sign_shape_small_norm_,
      0.0,
      1.0);
    const double configured_large_norm = std::clamp(
      cmd_vel_small_opposite_sign_shape_large_norm_,
      0.0,
      1.0);
    const double negative_norm = std::clamp(
      std::min(configured_small_norm, configured_large_norm),
      0.0,
      1.0);
    const double positive_norm = std::clamp(
      std::max(configured_small_norm, configured_large_norm),
      0.0,
      1.0);
    const double negative_step_m = negative_norm * max_step_length_m;
    const double positive_step_m = positive_norm * max_step_length_m;
    const bool ccw_turn = original_left_norm < 0.0 && original_right_norm > 0.0;
    command.step_command.left_step_length_m = ccw_turn ? -negative_step_m : positive_step_m;
    command.step_command.right_step_length_m = ccw_turn ? positive_step_m : -negative_step_m;
    command.step_command.forward_step_length_m = 0.5 * (
      command.step_command.left_step_length_m + command.step_command.right_step_length_m);
    command.step_command.step_yaw_rad =
      (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
      std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
    command.min_output_step_norm_applied = true;
    command.opposite_sign_turn_shape_applied = true;
    return;
  }
  if (small_opposite_sign_step_override)
  {
    // Runtime yaw-recovery/alignment overrides use small opposite-sign steps.
    // Flooring them to the mode minimum turns a fine correction into a hard spin.
    return;
  }
  bool adjusted = false;

  const auto floor_nonzero_step = [&](double step_m) {
    if (std::abs(step_m) <= 1e-6 || std::abs(step_m) >= min_abs_step_m) {
      return step_m;
    }
    adjusted = true;
    return std::copysign(min_abs_step_m, step_m);
  };

  double left_step_m = command.step_command.left_step_length_m;
  double right_step_m = command.step_command.right_step_length_m;
  if (uphill_segment_floor_active || uphill_pitch_floor_active) {
    // Uphill speed floors must not erase path/yaw correction. Add the same
    // forward bias to both sides so the slower side reaches the floor while
    // preserving as much left/right differential as the output clamp allows.
    const double min_side_step_m = std::min(left_step_m, right_step_m);
    if (min_side_step_m < min_abs_step_m) {
      const double common_bias_m = min_abs_step_m - min_side_step_m;
      left_step_m += common_bias_m;
      right_step_m += common_bias_m;
      adjusted = true;
    }
  } else {
    left_step_m = floor_nonzero_step(left_step_m);
    right_step_m = floor_nonzero_step(right_step_m);
  }
  if (!adjusted) {
    return;
  }

  command.step_command.left_step_length_m = std::clamp(
    left_step_m,
    -max_step_length_m,
    max_step_length_m);
  command.step_command.right_step_length_m = std::clamp(
    right_step_m,
    -max_step_length_m,
    max_step_length_m);
  command.step_command.forward_step_length_m = 0.5 * (
    command.step_command.left_step_length_m + command.step_command.right_step_length_m);
  command.step_command.step_yaw_rad =
    (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.min_output_step_norm_applied = true;
}

void CmdVelToSerialNode::applyOppositeSignTurnShape(TimedStepCommand & command) const
{
  if (!opposite_sign_turn_shape_enabled_) {
    return;
  }
  if (!modeInList(command.serial_mode, opposite_sign_turn_shape_modes_)) {
    return;
  }

  const double left_step_m = command.step_command.left_step_length_m;
  const double right_step_m = command.step_command.right_step_length_m;
  if (std::abs(left_step_m) <= 1e-6 || std::abs(right_step_m) <= 1e-6) {
    return;
  }
  if (left_step_m * right_step_m >= 0.0) {
    return;
  }

  const double small_norm = std::clamp(opposite_sign_turn_shape_small_norm_, 0.0, 1.0);
  const double large_norm = std::clamp(
    std::max(opposite_sign_turn_shape_large_norm_, small_norm + 1e-3),
    0.0,
    1.0);
  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  const double small_step_m = small_norm * max_step_length_m;
  const double large_step_m = large_norm * max_step_length_m;

  const bool ccw_turn = left_step_m < 0.0 && right_step_m > 0.0;
  command.step_command.left_step_length_m = ccw_turn ? -small_step_m : large_step_m;
  command.step_command.right_step_length_m = ccw_turn ? large_step_m : -small_step_m;
  command.step_command.forward_step_length_m = 0.5 * (
    command.step_command.left_step_length_m + command.step_command.right_step_length_m);
  command.step_command.step_yaw_rad =
    (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);
  command.opposite_sign_turn_shape_applied = true;
}

void CmdVelToSerialNode::applyOutputStepSlewLimit(TimedStepCommand & command) const
{
  if (!output_step_slew_limit_enabled_ || output_step_slew_limit_max_delta_norm_ <= 0.0) {
    output_step_slew_cache_valid_ = false;
    return;
  }
  if (!modeInList(command.serial_mode, output_step_slew_limit_modes_)) {
    output_step_slew_cache_valid_ = false;
    return;
  }

  const double max_step_length_m = mapping_parameters_.max_step_length_m;
  if (max_step_length_m <= 1e-6) {
    output_step_slew_cache_valid_ = false;
    return;
  }

  const double target_left_norm = normalizeStepLength(command.step_command.left_step_length_m);
  const double target_right_norm = normalizeStepLength(command.step_command.right_step_length_m);
  if (std::abs(target_left_norm) <= 1e-6 && std::abs(target_right_norm) <= 1e-6) {
    output_step_slew_cache_valid_ = false;
    return;
  }

  if (
    !output_step_slew_cache_valid_ ||
    last_output_step_slew_mode_ != command.serial_mode)
  {
    last_output_step_slew_mode_ = command.serial_mode;
    last_output_step_slew_left_norm_ = target_left_norm;
    last_output_step_slew_right_norm_ = target_right_norm;
    output_step_slew_cache_valid_ = true;
    return;
  }

  const auto limit_delta = [this](double target, double previous) {
      const double delta = target - previous;
      if (std::abs(delta) <= output_step_slew_limit_max_delta_norm_) {
        return target;
      }
      return previous + std::copysign(output_step_slew_limit_max_delta_norm_, delta);
    };

  const double limited_left_norm = std::clamp(
    limit_delta(target_left_norm, last_output_step_slew_left_norm_), -1.0, 1.0);
  const double limited_right_norm = std::clamp(
    limit_delta(target_right_norm, last_output_step_slew_right_norm_), -1.0, 1.0);

  command.step_command.left_step_length_m = limited_left_norm * max_step_length_m;
  command.step_command.right_step_length_m = limited_right_norm * max_step_length_m;
  command.step_command.forward_step_length_m = 0.5 * (
    command.step_command.left_step_length_m + command.step_command.right_step_length_m);
  command.step_command.step_yaw_rad =
    (command.step_command.right_step_length_m - command.step_command.left_step_length_m) /
    std::max(2.0 * mapping_parameters_.stance_half_width_m, 1e-6);

  last_output_step_slew_left_norm_ = limited_left_norm;
  last_output_step_slew_right_norm_ = limited_right_norm;
}

bool CmdVelToSerialNode::minOutputStepNormForMode(int mode, double & min_norm) const
{
  for (std::size_t idx = 0; idx < min_output_step_norm_override_modes_.size(); ++idx) {
    if (min_output_step_norm_override_modes_[idx] == static_cast<int64_t>(mode)) {
      min_norm = min_output_step_norm_override_norms_[idx];
      return true;
    }
  }

  if (!modeInList(mode, min_output_step_norm_modes_)) {
    return false;
  }

  min_norm = min_output_step_norm_;
  return true;
}

bool CmdVelToSerialNode::uphillSegmentStepFloorForCommand(
  const TimedStepCommand & command,
  const rclcpp::Time & stamp,
  double & min_norm) const
{
  if (!uphill_segment_step_floor_enabled_ || !uphill_step_floor_segment_enabled_) {
    return false;
  }
  if (!received_odom_) {
    return false;
  }
  if (!modeInList(command.serial_mode, uphill_segment_step_floor_modes_)) {
    return false;
  }
  if (
    uphill_segment_step_floor_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > uphill_segment_step_floor_stale_timeout_sec_)
  {
    return false;
  }
  if (
    uphill_segment_step_floor_forward_only_ &&
    command.step_command.forward_step_length_m <= 1e-6)
  {
    return false;
  }
  if (
    uphill_segment_step_floor_max_abs_wz_rps_ > 0.0 &&
    std::abs(command.wz) > uphill_segment_step_floor_max_abs_wz_rps_)
  {
    return false;
  }
  if (received_goal_ && uphill_segment_step_floor_goal_distance_min_m_ > 0.0) {
    const double goal_distance_m = std::hypot(
      latest_goal_x_ - latest_odom_x_,
      latest_goal_y_ - latest_odom_y_);
    if (goal_distance_m < uphill_segment_step_floor_goal_distance_min_m_) {
      return false;
    }
  }
  if (uphill_segment_step_floor_min_norm_ <= 1e-6) {
    return false;
  }

  min_norm = uphill_segment_step_floor_min_norm_;
  return true;
}

bool CmdVelToSerialNode::uphillPitchStepFloorForCommand(
  const TimedStepCommand & command,
  const rclcpp::Time & stamp,
  double & min_norm) const
{
  if (!uphill_pitch_step_floor_enabled_) {
    return false;
  }
  if (uphill_pitch_step_floor_require_segment_enable_ && !uphill_step_floor_segment_enabled_) {
    return false;
  }
  if (!received_odom_) {
    return false;
  }
  if (!modeInList(command.serial_mode, uphill_pitch_step_floor_modes_)) {
    return false;
  }
  if (
    uphill_pitch_step_floor_stale_timeout_sec_ > 0.0 &&
    (stamp - last_odom_stamp_).seconds() > uphill_pitch_step_floor_stale_timeout_sec_)
  {
    return false;
  }
  if (
    uphill_pitch_step_floor_forward_only_ &&
    command.step_command.forward_step_length_m <= 1e-6)
  {
    return false;
  }
  // 2026-05-01: old logic also applied the 0.75 floor during large Nav2 turns,
  // turning an angular command into [0.75,0.75] straight walking and causing
  // alternating straight/turn outputs. Keep the speed floor for uphill straight
  // climbing only; let Nav2 own high-wz heading corrections.
  if (
    uphill_pitch_step_floor_max_abs_wz_rps_ > 0.0 &&
    std::abs(command.wz) > uphill_pitch_step_floor_max_abs_wz_rps_)
  {
    return false;
  }

  // 2026-05-01: with the 90-degree rotated lidar/body frame, ramp tilt appears
  // on odom roll instead of pitch, so reuse the existing threshold with roll.
  const double roll_for_detection =
    uphill_pitch_step_floor_use_abs_pitch_ ? std::abs(latest_odom_roll_) : latest_odom_roll_;
  if (roll_for_detection < uphill_pitch_step_floor_min_pitch_rad_) {
    return false;
  }

  min_norm = uphill_pitch_step_floor_min_norm_;
  return true;
}

bool CmdVelToSerialNode::ensureSerialOpen(const rclcpp::Time & now)
{
  if (serial_fd_ >= 0) {
    return true;
  }

  if ((now - last_open_attempt_).seconds() < reconnect_interval_sec_) {
    return false;
  }

  last_open_attempt_ = now;
  serial_fd_ = ::open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Failed opening %s: %s",
      serial_device_.c_str(),
      std::strerror(errno));
    publishSerialConnected(false);
    return false;
  }

  if (!configureSerialPort(serial_fd_)) {
    closeSerialPort();
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Opened serial device %s", serial_device_.c_str());
  publishSerialConnected(true);
  return true;
}

void CmdVelToSerialNode::closeSerialPort()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
    publishSerialConnected(false);
  }
}

void CmdVelToSerialNode::publishModeEcho() const
{
  if (!mode_echo_pub_) {
    return;
  }

  std_msgs::msg::Int32 msg;
  msg.data = mode_;
  mode_echo_pub_->publish(msg);
}

void CmdVelToSerialNode::publishSerialConnected(bool connected)
{
  if (!serial_connected_pub_ || connected == last_serial_connected_) {
    return;
  }

  last_serial_connected_ = connected;
  std_msgs::msg::Bool msg;
  msg.data = connected;
  serial_connected_pub_->publish(msg);
}

bool CmdVelToSerialNode::configureSerialPort(int fd) const
{
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    return false;
  }

  const auto baud_constant = baudRateToConstant(baud_rate_);
  cfsetospeed(&tty, baud_constant);
  cfsetispeed(&tty, baud_constant);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~IGNBRK;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag &= ~(INLCR | ICRNL);
  tty.c_oflag &= ~(ONLCR | OCRNL);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    return false;
  }

  return true;
}

std::string CmdVelToSerialNode::formatSerialMessage(
  const StepCommand & step_command,
  int serial_mode) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(output_precision_)
         << message_prefix_
         << serial_mode
         << field_separator_
         << normalizeStepLength(step_command.left_step_length_m)
         << field_separator_
         << normalizeStepLength(step_command.right_step_length_m)
         << message_suffix_;
  return stream.str();
}

std::string CmdVelToSerialNode::formatNormalizedSerialMessage(
  int serial_mode,
  double left_norm,
  double right_norm) const
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(output_precision_)
         << message_prefix_
         << serial_mode
         << field_separator_
         << std::clamp(left_norm, -1.0, 1.0)
         << field_separator_
         << std::clamp(right_norm, -1.0, 1.0)
         << message_suffix_;
  return stream.str();
}

double CmdVelToSerialNode::normalizeStepLength(double step_length_m) const
{
  return StepLengthMapper::normalizeSignedRatio(
    step_length_m, mapping_parameters_.max_step_length_m);
}

bool CmdVelToSerialNode::writeSerialMessage(
  const std::string & message,
  const rclcpp::Time & now,
  bool log_message)
{
  std::size_t total_bytes_written = 0;
  while (total_bytes_written < message.size()) {
    const auto bytes_written = ::write(
      serial_fd_,
      message.data() + total_bytes_written,
      message.size() - total_bytes_written);
    if (bytes_written < 0) {
      if (errno == EINTR) {
        continue;
      }

      RCLCPP_ERROR(
        this->get_logger(),
        "Failed writing to %s: %s",
        serial_device_.c_str(),
        std::strerror(errno));
      closeSerialPort();
      return false;
    }

    if (bytes_written == 0) {
      RCLCPP_ERROR(this->get_logger(), "Serial write returned 0 bytes for %s", serial_device_.c_str());
      closeSerialPort();
      return false;
    }

    total_bytes_written += static_cast<std::size_t>(bytes_written);
  }

  if (log_message && shouldLogSerialMessage(message, now)) {
    RCLCPP_INFO(
      this->get_logger(),
      "%sserial tx: %s%s",
      styleOrEmpty(kAnsiSerial),
      message.c_str(),
      resetOrEmpty());
    updateSerialLogCache(message, now);
  }
  return true;
}

bool CmdVelToSerialNode::shouldLogSerialMessage(const std::string & message, const rclcpp::Time & now) const
{
  int serial_mode = 0;
  double left_norm = 0.0;
  double right_norm = 0.0;
  const bool parsed = parseSerialLogMessage(message, serial_mode, left_norm, right_norm);
  if (!last_serial_log_values_valid_ && last_serial_log_message_.empty()) {
    return true;
  }
  if (!parsed || !last_serial_log_values_valid_) {
    if (message != last_serial_log_message_) {
      return true;
    }
    return (now - last_serial_log_time_).seconds() >= serial_log_min_interval_sec_;
  }
  const bool mode_changed = serial_mode != last_serial_log_mode_;
  const bool left_changed = std::fabs(left_norm - last_serial_log_left_norm_) >=
    serial_log_change_threshold_;
  const bool right_changed = std::fabs(right_norm - last_serial_log_right_norm_) >=
    serial_log_change_threshold_;
  const bool direction_changed =
    (left_norm * last_serial_log_left_norm_ < 0.0) ||
    (right_norm * last_serial_log_right_norm_ < 0.0);
  if (mode_changed || left_changed || right_changed || direction_changed) {
    return true;
  }
  return (now - last_serial_log_time_).seconds() >= serial_log_min_interval_sec_;
}

bool CmdVelToSerialNode::parseSerialLogMessage(
  const std::string & message,
  int & serial_mode,
  double & left_norm,
  double & right_norm) const
{
  const auto start = message.find(message_prefix_);
  if (start == std::string::npos) {
    return false;
  }
  const auto end = message.find(message_suffix_, start + message_prefix_.size());
  if (end == std::string::npos) {
    return false;
  }
  const auto payload = message.substr(
    start + message_prefix_.size(),
    end - start - message_prefix_.size());
  const auto first_sep = payload.find(field_separator_);
  if (first_sep == std::string::npos) {
    return false;
  }
  const auto second_sep = payload.find(field_separator_, first_sep + field_separator_.size());
  if (second_sep == std::string::npos) {
    return false;
  }
  try {
    serial_mode = std::stoi(payload.substr(0, first_sep));
    left_norm = std::stod(payload.substr(
      first_sep + field_separator_.size(),
      second_sep - first_sep - field_separator_.size()));
    right_norm = std::stod(payload.substr(second_sep + field_separator_.size()));
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

bool CmdVelToSerialNode::shouldLogStepDebug(
  const TimedStepCommand & timed_step_command,
  double left_norm,
  double right_norm,
  const rclcpp::Time & now) const
{
  if (last_logged_mode_ == -999) {
    return true;
  }
  const bool mode_changed = timed_step_command.serial_mode != last_logged_mode_;
  const bool left_changed = std::fabs(left_norm - last_logged_left_norm_) >= step_log_change_threshold_;
  const bool right_changed = std::fabs(right_norm - last_logged_right_norm_) >= step_log_change_threshold_;
  const bool vx_changed = std::fabs(timed_step_command.vx - last_logged_vx_) >= step_log_change_threshold_;
  const bool wz_changed = std::fabs(timed_step_command.wz - last_logged_wz_) >= step_log_change_threshold_;
  const bool interval_elapsed = (now - last_step_log_time_).seconds() >= step_debug_log_interval_sec_;
  return mode_changed || left_changed || right_changed || vx_changed || wz_changed || interval_elapsed;
}

void CmdVelToSerialNode::updateSerialLogCache(const std::string & message, const rclcpp::Time & now)
{
  last_serial_log_message_ = message;
  last_serial_log_time_ = now;
  int serial_mode = 0;
  double left_norm = 0.0;
  double right_norm = 0.0;
  last_serial_log_values_valid_ = parseSerialLogMessage(
    message, serial_mode, left_norm, right_norm);
  if (last_serial_log_values_valid_) {
    last_serial_log_mode_ = serial_mode;
    last_serial_log_left_norm_ = left_norm;
    last_serial_log_right_norm_ = right_norm;
  }
}

void CmdVelToSerialNode::updateStepLogCache(
  const TimedStepCommand & timed_step_command,
  double left_norm,
  double right_norm,
  const rclcpp::Time & now)
{
  last_logged_mode_ = timed_step_command.serial_mode;
  last_logged_left_norm_ = left_norm;
  last_logged_right_norm_ = right_norm;
  last_logged_vx_ = timed_step_command.vx;
  last_logged_wz_ = timed_step_command.wz;
  last_step_log_time_ = now;
}

const CmdVelToSerialNode::FixedStepOverrideConfig *
CmdVelToSerialNode::findFixedStepOverride(int mode) const
{
  for (const auto & fixed_step_override : fixed_step_overrides_) {
    if (fixed_step_override.mode == mode) {
      return &fixed_step_override;
    }
  }
  return nullptr;
}

speed_t CmdVelToSerialNode::baudRateToConstant(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::runtime_error("Unsupported baud_rate. Supported: 9600, 19200, 38400, 57600, 115200, 230400.");
  }
}

bool CmdVelToSerialNode::modeInList(int mode, const std::vector<int64_t> & mode_list)
{
  return std::find(mode_list.begin(), mode_list.end(), static_cast<int64_t>(mode)) != mode_list.end();
}

std::string CmdVelToSerialNode::trimAscii(const std::string & value)
{
  std::size_t begin = 0;
  while (
    begin < value.size() &&
    std::isspace(static_cast<unsigned char>(value[begin])) != 0)
  {
    ++begin;
  }

  std::size_t end = value.size();
  while (
    end > begin &&
    std::isspace(static_cast<unsigned char>(value[end - 1])) != 0)
  {
    --end;
  }

  return value.substr(begin, end - begin);
}

std::string CmdVelToSerialNode::toLowerAscii(const std::string & value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lowered;
}

double CmdVelToSerialNode::controllerYaw() const
{
  return normalizeAngle(latest_odom_yaw_ + pose_yaw_offset_rad_);
}

}  // namespace quadruped_step_mapper
