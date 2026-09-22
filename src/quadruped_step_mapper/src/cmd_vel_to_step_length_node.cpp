#include "quadruped_step_mapper/cmd_vel_to_step_length_node.hpp"

#include <chrono>
#include <stdexcept>

namespace quadruped_step_mapper
{

CmdVelToStepLengthNode::CmdVelToStepLengthNode(const rclcpp::NodeOptions & options)
: Node("cmd_vel_to_step_length", options)
{
  last_cmd_stamp_ = this->get_clock()->now();

  const auto input_cmd_vel_topic =
    this->declare_parameter<std::string>("input_cmd_vel_topic", "/cmd_vel");
  const auto output_step_length_topic =
    this->declare_parameter<std::string>("output_step_length_topic", "/step_length_cmd");
  const auto output_step_length_twist_topic =
    this->declare_parameter<std::string>("output_step_length_twist_topic", "/step_length_twist");

  output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "body");
  mapping_parameters_.step_frequency_hz =
    this->declare_parameter<double>("step_frequency_hz", 27.0);
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 27.0);
  mapping_parameters_.stance_half_width_m =
    this->declare_parameter<double>("stance_half_width_m", 0.18);
  mapping_parameters_.max_step_length_m =
    this->declare_parameter<double>("max_step_length_m", 0.03704);
  mapping_parameters_.max_step_yaw_rad =
    this->declare_parameter<double>("max_step_yaw_rad", 0.35);
  mapping_parameters_.max_linear_speed_mps =
    this->declare_parameter<double>("max_linear_speed_mps", 1.0);
  mapping_parameters_.max_angular_speed_rps =
    this->declare_parameter<double>("max_angular_speed_rps", 1.20);
  mapping_parameters_.min_effective_linear_speed_mps =
    this->declare_parameter<double>("min_effective_linear_speed_mps", 0.01);
  mapping_parameters_.min_effective_angular_speed_rps =
    this->declare_parameter<double>("min_effective_angular_speed_rps", 0.05);
  cmd_vel_timeout_sec_ = this->declare_parameter<double>("cmd_vel_timeout_sec", 2.0);
  mapping_parameters_.scale_to_limits = this->declare_parameter<bool>("scale_to_limits", true);
  publish_zero_on_timeout_ = this->declare_parameter<bool>("publish_zero_on_timeout", true);

  if (publish_rate_hz_ <= 0.0) {
    throw std::runtime_error("publish_rate_hz must be > 0.");
  }
  mapper_ = std::make_unique<StepLengthMapper>(mapping_parameters_);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic,
    rclcpp::SystemDefaultsQoS(),
    std::bind(&CmdVelToStepLengthNode::cmdVelCallback, this, std::placeholders::_1));

  step_length_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
    output_step_length_topic, rclcpp::SystemDefaultsQoS());
  step_length_twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    output_step_length_twist_topic, rclcpp::SystemDefaultsQoS());

  const auto publish_period =
    std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(publish_period),
    std::bind(&CmdVelToStepLengthNode::publishStepCommand, this));

  RCLCPP_INFO(
    this->get_logger(),
    "cmd_vel->step_length mapper started. step_frequency=%.3f Hz, stance_half_width=%.3f m, "
    "max_step_length=%.3f m, max_step_yaw=%.3f rad.",
    mapping_parameters_.step_frequency_hz,
    mapping_parameters_.stance_half_width_m,
    mapping_parameters_.max_step_length_m,
    mapping_parameters_.max_step_yaw_rad);
}

void CmdVelToStepLengthNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  latest_cmd_vel_ = *msg;
  last_cmd_stamp_ = this->get_clock()->now();
  received_cmd_ = true;
  RCLCPP_INFO_THROTTLE(
    this->get_logger(),
    *this->get_clock(),
    1000,
    "Received /cmd_vel: vx=%.3f m/s, wz=%.3f rad/s",
    msg->linear.x,
    msg->angular.z);
}

void CmdVelToStepLengthNode::publishStepCommand()
{
  const auto now = this->get_clock()->now();
  const auto timed_step_command = computeTimedStepCommand(now);

  if (timed_step_command.timed_out != last_timeout_state_) {
    if (timed_step_command.timed_out) {
      RCLCPP_WARN(
        this->get_logger(),
        "/cmd_vel timed out after %.3f s, publishing zero step command.",
        cmd_vel_timeout_sec_);
    } else {
      RCLCPP_INFO(this->get_logger(), "/cmd_vel stream is active, publishing mapped step command.");
    }
    last_timeout_state_ = timed_step_command.timed_out;
  }

  if (timed_step_command.timed_out && !publish_zero_on_timeout_) {
    return;
  }

  geometry_msgs::msg::Vector3Stamped step_length_msg;
  step_length_msg.header.stamp = now;
  step_length_msg.header.frame_id = output_frame_id_;
  step_length_msg.vector.x = timed_step_command.step_command.left_step_length_m;
  step_length_msg.vector.y = timed_step_command.step_command.right_step_length_m;
  step_length_msg.vector.z = timed_step_command.step_command.step_yaw_rad;
  step_length_pub_->publish(step_length_msg);

  geometry_msgs::msg::TwistStamped step_length_twist_msg;
  step_length_twist_msg.header = step_length_msg.header;
  step_length_twist_msg.twist.linear.x = timed_step_command.step_command.forward_step_length_m;
  step_length_twist_msg.twist.angular.z = timed_step_command.step_command.step_yaw_rad;
  step_length_twist_pub_->publish(step_length_twist_msg);
}

CmdVelToStepLengthNode::TimedStepCommand
CmdVelToStepLengthNode::computeTimedStepCommand(const rclcpp::Time & stamp) const
{
  TimedStepCommand command;

  const auto cmd_age_sec = (stamp - last_cmd_stamp_).seconds();
  const bool stale = !received_cmd_ || cmd_age_sec > cmd_vel_timeout_sec_;

  double vx = latest_cmd_vel_.linear.x;
  double wz = latest_cmd_vel_.angular.z;

  if (stale) {
    command.timed_out = true;
    vx = 0.0;
    wz = 0.0;
  }

  command.step_command = mapper_->compute(vx, wz);
  return command;
}

}  // namespace quadruped_step_mapper
