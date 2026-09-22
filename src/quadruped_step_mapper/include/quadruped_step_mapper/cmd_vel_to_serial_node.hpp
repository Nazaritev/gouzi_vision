#ifndef QUADRUPED_STEP_MAPPER__CMD_VEL_TO_SERIAL_NODE_HPP_
#define QUADRUPED_STEP_MAPPER__CMD_VEL_TO_SERIAL_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <termios.h>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "quadruped_step_mapper/step_length_mapper.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quadruped_step_mapper
{

class CmdVelToSerialNode : public rclcpp::Node
{
public:
  explicit CmdVelToSerialNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CmdVelToSerialNode() override;

private:
  struct FixedStepOverrideConfig
  {
    int mode{0};
    double left_norm{0.0};
    double right_norm{0.0};
  };

  struct TimedStepCommand
  {
    StepCommand step_command{};
    int serial_mode{0};
    bool timed_out{false};
    double vx{0.0};
    double wz{0.0};
    double odom_y{0.0};
    double goal_y{0.0};
    double goal_y_error{0.0};
    double odom_roll_rad{0.0};
    double odom_y_correction_norm{0.0};
    bool odom_y_correction_applied{false};
    double path_lateral_error{0.0};
    double path_lateral_correction_norm{0.0};
    bool path_lateral_correction_applied{false};
    double path_yaw_error_rad{0.0};
    double path_yaw_correction_norm{0.0};
    bool path_yaw_correction_applied{false};
    int path_correction_profile{1};
    double crouch_yaw_error_rad{0.0};
    double crouch_yaw_correction_norm{0.0};
    bool crouch_yaw_correction_applied{false};
    double crouch_forward_bias_norm{0.0};
    bool crouch_forward_bias_applied{false};
    bool same_direction_turn_applied{false};
    bool opposite_sign_turn_shape_applied{false};
    bool uphill_step_floor_segment_enabled{false};
    bool uphill_segment_step_floor_active{false};
    bool uphill_pitch_step_floor_active{false};
    bool min_output_step_norm_applied{false};
    bool step_override_applied{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void modeCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathReferenceStartCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void pathReferenceCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void updatePathReference(const geometry_msgs::msg::PoseStamped & msg, bool use_pending_start);
  void stepOverrideCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void stepOnceCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void uphillStepFloorStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void pathCorrectionProfileCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void fixedStepOverrideStateCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void transmitStepCommand();
  void readSerialInput(const rclcpp::Time & now);
  void handleSerialRxBytes(const char * data, std::size_t size, const rclcpp::Time & now);
  void handleSerialRxFrame(const std::string & frame, const rclcpp::Time & now);
  void publishStartSignal(const rclcpp::Time & now);
  void publishFinishSignal(const rclcpp::Time & now);
  void publishActionAck(const std::string & token, const rclcpp::Time & now);
  TimedStepCommand computeTimedStepCommand(const rclcpp::Time & stamp) const;
  void applyOdomYCorrection(TimedStepCommand & command, const rclcpp::Time & stamp) const;
  void applyPathLateralCorrection(TimedStepCommand & command, const rclcpp::Time & stamp) const;
  void applyPathYawCorrection(TimedStepCommand & command, const rclcpp::Time & stamp) const;
  void applyCrouchYawCorrection(TimedStepCommand & command, const rclcpp::Time & stamp) const;
  void applyCrouchForwardBias(TimedStepCommand & command) const;
  void applySameDirectionTurnBias(TimedStepCommand & command) const;
  void applyMinOutputStepNorm(TimedStepCommand & command, const rclcpp::Time & stamp) const;
  void applyOppositeSignTurnShape(TimedStepCommand & command) const;
  void applyOutputStepSlewLimit(TimedStepCommand & command) const;
  bool minOutputStepNormForMode(int mode, double & min_norm) const;
  bool uphillSegmentStepFloorForCommand(
    const TimedStepCommand & command,
    const rclcpp::Time & stamp,
    double & min_norm) const;
  bool uphillPitchStepFloorForCommand(
    const TimedStepCommand & command,
    const rclcpp::Time & stamp,
    double & min_norm) const;
  bool ensureSerialOpen(const rclcpp::Time & now);
  void closeSerialPort();
  void publishModeEcho() const;
  void publishSerialConnected(bool connected);
  bool configureSerialPort(int fd) const;
  std::string formatSerialMessage(const StepCommand & step_command, int serial_mode) const;
  std::string formatNormalizedSerialMessage(int serial_mode, double left_norm, double right_norm) const;
  double normalizeStepLength(double step_length_m) const;
  bool writeSerialMessage(const std::string & message, const rclcpp::Time & now, bool log_message);
  bool shouldLogSerialMessage(const std::string & message, const rclcpp::Time & now) const;
  bool parseSerialLogMessage(
    const std::string & message,
    int & serial_mode,
    double & left_norm,
    double & right_norm) const;
  bool shouldLogStepDebug(
    const TimedStepCommand & timed_step_command,
    double left_norm,
    double right_norm,
    const rclcpp::Time & now) const;
  const FixedStepOverrideConfig * findFixedStepOverride(int mode) const;
  void updateSerialLogCache(const std::string & message, const rclcpp::Time & now);
  void updateStepLogCache(
    const TimedStepCommand & timed_step_command,
    double left_norm,
    double right_norm,
    const rclcpp::Time & now);
  static speed_t baudRateToConstant(int baud_rate);
  static bool modeInList(int mode, const std::vector<int64_t> & mode_list);
  static std::string trimAscii(const std::string & value);
  static std::string toLowerAscii(const std::string & value);
  double controllerYaw() const;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr path_reference_start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr path_reference_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr step_override_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr step_once_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr uphill_step_floor_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr path_correction_profile_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fixed_step_override_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_echo_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr serial_connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr start_signal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_signal_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_ack_pub_;
  rclcpp::TimerBase::SharedPtr transmit_timer_;

  geometry_msgs::msg::Twist latest_cmd_vel_{};
  rclcpp::Time last_cmd_stamp_;
  bool received_cmd_{false};

  StepMappingParameters mapping_parameters_{};
  std::unique_ptr<StepLengthMapper> mapper_;

  std::string serial_device_;
  int baud_rate_{115200};
  int serial_fd_{-1};
  bool start_signal_rx_enabled_{false};
  std::string start_signal_topic_;
  std::string start_signal_token_;
  std::string finish_signal_topic_;
  std::string finish_signal_token_;
  std::string action_ack_topic_;
  std::string jumping_signal_token_;
  bool start_signal_case_sensitive_{false};
  bool start_signal_received_{false};
  bool finish_signal_received_{false};
  bool log_serial_rx_frames_{false};
  std::size_t serial_rx_max_frame_len_{128};
  std::string serial_rx_frame_buffer_;
  bool serial_rx_in_frame_{false};
  rclcpp::Time last_open_attempt_;
  double reconnect_interval_sec_{1.0};
  double transmit_rate_hz_{30.0};
  double cmd_vel_timeout_sec_{0.50};
  int mode_{0};
  bool fixed_step_override_enabled_{false};
  bool fixed_step_override_runtime_enabled_{true};
  int fixed_step_override_mode_{1};
  double fixed_step_override_left_norm_{1.0};
  double fixed_step_override_right_norm_{1.0};
  std::vector<FixedStepOverrideConfig> fixed_step_overrides_;
  std::string goal_topic_;
  std::string path_reference_start_topic_;
  std::string path_reference_topic_;
  std::string odom_topic_;
  bool odom_y_correction_enabled_{false};
  bool odom_y_correction_use_goal_y_{true};
  std::vector<int64_t> odom_y_correction_modes_;
  double odom_y_correction_deadband_m_{0.05};
  double odom_y_correction_gain_norm_per_m_{0.0};
  double odom_y_correction_max_norm_{0.0};
  bool odom_y_correction_positive_only_{true};
  double odom_y_correction_stale_timeout_sec_{0.50};
  double odom_y_correction_goal_distance_min_m_{0.25};
  double odom_y_correction_max_abs_wz_rps_{0.35};
  bool path_lateral_correction_enabled_{false};
  std::vector<int64_t> path_lateral_correction_modes_;
  std::vector<int64_t> path_lateral_direct_override_modes_;
  double path_lateral_correction_deadband_m_{0.05};
  double path_lateral_correction_gain_norm_per_m_{0.0};
  double path_lateral_correction_max_norm_{0.0};
  double path_lateral_correction_sign_{1.0};
  double path_lateral_direct_high_norm_{1.0};
  double path_lateral_direct_low_norm_{0.2};
  double path_lateral_correction_stale_timeout_sec_{0.50};
  double path_lateral_correction_goal_distance_min_m_{0.25};
  double path_lateral_correction_max_abs_wz_rps_{0.35};
  double path_lateral_correction_min_segment_length_m_{0.30};
  bool uphill_path_lateral_correction_enabled_{false};
  double uphill_path_lateral_correction_deadband_m_{0.03};
  double uphill_path_lateral_correction_gain_norm_per_m_{0.0};
  double uphill_path_lateral_correction_max_norm_{0.0};
  double uphill_path_lateral_correction_sign_{1.0};
  double uphill_path_lateral_correction_goal_distance_min_m_{0.15};
  double uphill_path_lateral_correction_max_abs_wz_rps_{0.80};
  bool path_yaw_correction_enabled_{false};
  std::vector<int64_t> path_yaw_correction_modes_;
  std::vector<int64_t> path_yaw_direct_override_modes_;
  double path_yaw_correction_deadband_rad_{0.0};
  double path_yaw_correction_gain_norm_per_rad_{0.0};
  double path_yaw_correction_max_norm_{0.0};
  double path_yaw_direct_high_norm_{1.0};
  double path_yaw_direct_low_norm_{0.2};
  double path_yaw_correction_stale_timeout_sec_{0.50};
  double path_yaw_correction_goal_distance_min_m_{0.25};
  double path_yaw_correction_max_abs_wz_rps_{0.35};
  double path_yaw_correction_min_segment_length_m_{0.30};
  int path_correction_profile_{1};
  std::string path_correction_profile_topic_;
  double path_correction_new_goal_delay_sec_{0.0};
  bool uphill_path_yaw_correction_enabled_{false};
  std::vector<int64_t> uphill_path_yaw_correction_modes_;
  double uphill_path_yaw_correction_deadband_rad_{0.0};
  double uphill_path_yaw_correction_gain_norm_per_rad_{0.0};
  double uphill_path_yaw_correction_max_norm_{0.0};
  double uphill_path_yaw_correction_goal_distance_min_m_{0.20};
  double uphill_path_yaw_correction_max_abs_wz_rps_{0.80};
  double latest_odom_x_{0.0};
  double latest_odom_y_{0.0};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  bool received_odom_{false};
  double latest_goal_x_{0.0};
  double latest_goal_y_{0.0};
  bool received_goal_{false};
  rclcpp::Time last_goal_stamp_{0, 0, RCL_ROS_TIME};
  double path_lateral_start_x_{0.0};
  double path_lateral_start_y_{0.0};
  bool path_lateral_start_captured_{false};
  double pending_path_reference_start_x_{0.0};
  double pending_path_reference_start_y_{0.0};
  rclcpp::Time pending_path_reference_start_stamp_{0, 0, RCL_ROS_TIME};
  bool pending_path_reference_start_received_{false};
  double path_reference_start_timeout_sec_{0.50};
  double latest_odom_roll_{0.0};
  double latest_odom_yaw_{0.0};
  double latest_goal_yaw_{0.0};
  bool crouch_yaw_correction_enabled_{false};
  std::vector<int64_t> crouch_yaw_correction_modes_;
  double crouch_yaw_correction_deadband_rad_{0.0};
  double crouch_yaw_correction_gain_norm_per_rad_{0.0};
  double crouch_yaw_correction_max_norm_{0.0};
  bool crouch_yaw_initial_boost_enabled_{false};
  double crouch_yaw_initial_boost_duration_sec_{0.0};
  double crouch_yaw_initial_boost_progress_m_{0.0};
  double crouch_yaw_initial_boost_min_error_rad_{0.0};
  double crouch_yaw_initial_boost_max_norm_{0.0};
  bool crouch_yaw_lateral_priority_enabled_{false};
  double crouch_yaw_lateral_priority_error_threshold_m_{0.07};
  double crouch_yaw_lateral_priority_max_norm_{0.03};
  double crouch_yaw_correction_stale_timeout_sec_{0.50};
  double crouch_yaw_correction_goal_distance_min_m_{0.25};
  double crouch_yaw_correction_max_abs_wz_rps_{0.35};
  bool crouch_forward_bias_enabled_{false};
  std::vector<int64_t> crouch_forward_bias_modes_;
  double crouch_forward_bias_norm_{0.0};
  double crouch_forward_bias_min_forward_step_norm_{0.0};
  double crouch_forward_bias_max_abs_wz_rps_{0.35};
  bool same_direction_turn_enabled_{false};
  std::vector<int64_t> same_direction_turn_modes_;
  double same_direction_turn_min_wz_rps_{0.15};
  double same_direction_turn_min_norm_{0.02};
  bool opposite_sign_turn_shape_enabled_{false};
  std::vector<int64_t> opposite_sign_turn_shape_modes_;
  double opposite_sign_turn_shape_small_norm_{0.20};
  double opposite_sign_turn_shape_large_norm_{0.65};
  bool step_override_small_opposite_sign_shape_enabled_{false};
  std::vector<int64_t> step_override_small_opposite_sign_shape_modes_;
  double step_override_small_opposite_sign_shape_small_norm_{0.15};
  double step_override_small_opposite_sign_shape_large_norm_{0.20};
  bool cmd_vel_small_opposite_sign_shape_enabled_{false};
  std::vector<int64_t> cmd_vel_small_opposite_sign_shape_modes_;
  double cmd_vel_small_opposite_sign_shape_small_norm_{0.18};
  double cmd_vel_small_opposite_sign_shape_large_norm_{0.20};
  bool min_output_step_norm_enabled_{false};
  std::vector<int64_t> min_output_step_norm_modes_;
  double min_output_step_norm_{0.0};
  std::vector<int64_t> min_output_step_norm_override_modes_;
  std::vector<double> min_output_step_norm_override_norms_;
  bool uphill_segment_step_floor_enabled_{false};
  std::vector<int64_t> uphill_segment_step_floor_modes_;
  bool uphill_segment_step_floor_forward_only_{true};
  double uphill_segment_step_floor_min_norm_{0.0};
  double uphill_segment_step_floor_goal_distance_min_m_{0.0};
  double uphill_segment_step_floor_stale_timeout_sec_{0.50};
  double uphill_segment_step_floor_max_abs_wz_rps_{0.0};
  bool uphill_pitch_step_floor_enabled_{false};
  bool uphill_pitch_step_floor_require_segment_enable_{false};
  bool uphill_step_floor_segment_enabled_{false};
  std::vector<int64_t> uphill_pitch_step_floor_modes_;
  double uphill_pitch_step_floor_min_pitch_rad_{0.0};
  bool uphill_pitch_step_floor_use_abs_pitch_{true};
  bool uphill_pitch_step_floor_forward_only_{true};
  double uphill_pitch_step_floor_min_norm_{0.0};
  double uphill_pitch_step_floor_stale_timeout_sec_{0.50};
  double uphill_pitch_step_floor_max_abs_wz_rps_{0.0};
  std::string step_override_topic_;
  std::string step_once_topic_;
  std::string uphill_step_floor_state_topic_;
  std::string fixed_step_override_state_topic_;
  double pose_yaw_offset_rad_{0.0};
  double step_override_timeout_sec_{0.25};
  int step_override_mode_{0};
  double step_override_left_norm_{0.0};
  double step_override_right_norm_{0.0};
  rclcpp::Time last_step_override_stamp_{0, 0, RCL_ROS_TIME};
  bool received_step_override_{false};

  std::string mode_topic_;
  std::string mode_echo_topic_;
  std::string serial_connected_topic_;
  int output_precision_{3};
  bool send_zero_on_timeout_{true};
  bool log_serial_messages_{false};
  bool log_step_debug_{true};
  bool output_step_slew_limit_enabled_{false};
  std::vector<int64_t> output_step_slew_limit_modes_;
  double output_step_slew_limit_max_delta_norm_{0.0};
  double serial_log_min_interval_sec_{1.0};
  double step_debug_log_interval_sec_{1.0};
  double serial_log_change_threshold_{0.02};
  double step_log_change_threshold_{0.02};
  bool last_serial_connected_{false};

  mutable rclcpp::Time last_serial_log_time_;
  mutable rclcpp::Time last_step_log_time_;
  mutable std::string last_serial_log_message_;
  mutable bool last_serial_log_values_valid_{false};
  mutable int last_serial_log_mode_{-999};
  mutable double last_serial_log_left_norm_{999.0};
  mutable double last_serial_log_right_norm_{999.0};
  mutable int last_logged_mode_{-999};
  mutable double last_logged_left_norm_{999.0};
  mutable double last_logged_right_norm_{999.0};
  mutable double last_logged_vx_{999.0};
  mutable double last_logged_wz_{999.0};
  mutable bool output_step_slew_cache_valid_{false};
  mutable int last_output_step_slew_mode_{0};
  mutable double last_output_step_slew_left_norm_{0.0};
  mutable double last_output_step_slew_right_norm_{0.0};

  std::string message_prefix_;
  std::string field_separator_;
  std::string message_suffix_;
};

}  // namespace quadruped_step_mapper

#endif  // QUADRUPED_STEP_MAPPER__CMD_VEL_TO_SERIAL_NODE_HPP_
