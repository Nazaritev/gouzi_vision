#include "quadruped_step_mapper/cmd_vel_to_serial_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<quadruped_step_mapper::CmdVelToSerialNode>());
  rclcpp::shutdown();
  return 0;
}
