#ifndef QUADRUPED_STEP_MAPPER__CMD_VEL_TO_STEP_LENGTH_NODE_HPP_
#define QUADRUPED_STEP_MAPPER__CMD_VEL_TO_STEP_LENGTH_NODE_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "quadruped_step_mapper/step_length_mapper.hpp"
#include "rclcpp/rclcpp.hpp"

namespace quadruped_step_mapper
{

class CmdVelToStepLengthNode : public rclcpp::Node
{
public:
  explicit CmdVelToStepLengthNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct TimedStepCommand
  {
    StepCommand step_command{};
    bool timed_out{false};
  };

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishStepCommand();
  TimedStepCommand computeTimedStepCommand(const rclcpp::Time & stamp) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr step_length_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr step_length_twist_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  geometry_msgs::msg::Twist latest_cmd_vel_{};
  rclcpp::Time last_cmd_stamp_;
  bool received_cmd_{false};

  StepMappingParameters mapping_parameters_{};
  std::unique_ptr<StepLengthMapper> mapper_;
  std::string output_frame_id_;
  double publish_rate_hz_{30.0};
  double cmd_vel_timeout_sec_{2.0};
  bool publish_zero_on_timeout_{true};
  bool last_timeout_state_{true};
};

}  // namespace quadruped_step_mapper

#endif  // QUADRUPED_STEP_MAPPER__CMD_VEL_TO_STEP_LENGTH_NODE_HPP_
