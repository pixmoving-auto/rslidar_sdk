#include "manager/node_manager.hpp"
#include <rs_driver/macro/version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <thread>
#include <csignal>

namespace robosense
{
namespace lidar
{

class RSLidarSDKComponent : public rclcpp::Node
{
public:
  explicit RSLidarSDKComponent(const rclcpp::NodeOptions & options)
  : Node("rslidar_sdk_node_component", options)
  {
    // Print version information
    RCLCPP_INFO(this->get_logger(), "********************************************************");
    RCLCPP_INFO(this->get_logger(), "**********                                    **********");
    RCLCPP_INFO(this->get_logger(), "**********    RSLidar_SDK Version: v%d.%d.%d    **********",
                RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR, RSLIDAR_VERSION_PATCH);
    RCLCPP_INFO(this->get_logger(), "**********                                    **********");
    RCLCPP_INFO(this->get_logger(), "********************************************************");

    // Set up config path
    std::string config_path;
    
#ifdef RUN_IN_ROS_WORKSPACE
    config_path = ament_index_cpp::get_package_share_directory("rslidar_sdk");
#else
    config_path = (std::string)PROJECT_PATH;
#endif

    config_path += "/config/config.yaml";

    std::string path = this->declare_parameter<std::string>("config_path", "");
    if (!path.empty())
    {
      config_path = path;
    }

    YAML::Node config;
    try
    {
      config = YAML::LoadFile(config_path);
      RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Config loaded from PATH:");
      RCLCPP_INFO(this->get_logger(), "%s", config_path.c_str());
      RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "The format of config file %s is wrong. Please check (e.g. indentation).", 
                  config_path.c_str());
      throw std::runtime_error("Failed to load config file");
    }

    node_manager_ = std::make_shared<NodeManager>();
    node_manager_->init(config);
    node_manager_->start();
  }

private:
  std::shared_ptr<NodeManager> node_manager_;
};

} // namespace lidar
} // namespace robosense

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarSDKComponent)