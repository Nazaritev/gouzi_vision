#include <chrono>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class PcdMapPublisher final : public rclcpp::Node
{
public:
  PcdMapPublisher()
  : Node("pcd_map_publisher")
  {
    pcd_path_ = declare_parameter<std::string>("pcd_path", "");
    topic_ = declare_parameter<std::string>("topic", "/pcd_map");
    frame_id_ = declare_parameter<std::string>("frame_id", "camera_init");
    publish_period_ = declare_parameter<double>("publish_period", 1.0);

    if (pcd_path_.empty()) {
      throw std::runtime_error(
        "Parameter 'pcd_path' is empty. Pass pcd_path:=/absolute/path/to/map.pcd");
    }

    std::error_code ec;
    const auto path = std::filesystem::path(pcd_path_);
    if (!std::filesystem::exists(path, ec)) {
      throw std::runtime_error("PCD file does not exist: " + pcd_path_);
    }
    if (!std::filesystem::is_regular_file(path, ec)) {
      throw std::runtime_error("PCD path is not a regular file: " + pcd_path_);
    }

    pcl::PCLPointCloud2 pcl_cloud;
    if (pcl::io::loadPCDFile(pcd_path_, pcl_cloud) < 0) {
      throw std::runtime_error("Failed to read PCD file: " + pcd_path_);
    }

    pcl_conversions::fromPCL(pcl_cloud, cloud_msg_);
    cloud_msg_.header.frame_id = frame_id_;
    // A static map does not need the PCD file's original timestamp. The header
    // is refreshed before each publication so RViz and TF can consume it.
    cloud_msg_.height = pcl_cloud.height;
    cloud_msg_.width = pcl_cloud.width;
    cloud_msg_.is_bigendian = pcl_cloud.is_bigendian;
    cloud_msg_.is_dense = pcl_cloud.is_dense;

    // Transient-local keeps the last map sample for late-joining subscribers.
    // The timer also republishes periodically for RViz configurations that use
    // volatile durability.
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.reliable().transient_local();
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(publish_period_));
    if (period.count() <= 0) {
      throw std::runtime_error("Parameter 'publish_period' must be greater than zero");
    }
    timer_ = create_wall_timer(period, [this]() { publish_map(); });

    RCLCPP_INFO(
      get_logger(), "Loaded %u points from %s; publishing %s in frame '%s' every %.3f s",
      cloud_msg_.width * cloud_msg_.height, pcd_path_.c_str(), topic_.c_str(),
      frame_id_.c_str(), publish_period_);
    publish_map();
  }

private:
  void publish_map()
  {
    cloud_msg_.header.stamp = now();
    publisher_->publish(cloud_msg_);
  }

  std::string pcd_path_;
  std::string topic_;
  std::string frame_id_;
  double publish_period_{1.0};
  sensor_msgs::msg::PointCloud2 cloud_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<PcdMapPublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(rclcpp::get_logger("pcd_map_publisher"), "%s", ex.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
