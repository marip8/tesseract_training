#include <rclcpp/node.hpp>

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

class PlanningServer : public rclcpp::Node
{
public:
  PlanningServer()
    : rclcpp::Node("planning_server")
    , env_(std::make_shared<tesseract_environment::Environment>())
  {
    // Get the URDF and SRDF from parameter
    declare_parameter("robot_description", "");
    declare_parameter("robot_description_semantic", "");
    const std::string urdf = get_parameter("robot_description").as_string();
    const std::string srdf = get_parameter("robot_description_semantic").as_string();

    // Populate the environment from URDF and SRDF
    auto locator = std::make_shared<tesseract_rosutils::ROSResourceLocator>();
    if (!env_->init(urdf, srdf, locator))
      throw std::runtime_error("Failed to initialize environment");

    // Create and configure the environment monitor (i.e., connection to the ROS ecosystem)
    std::shared_ptr<rclcpp::Node> node_ptr(this, [](rclcpp::Node*){});

    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_ptr, env_, "samxl");
    monitor_->startStateMonitor("/joint_states", false);
    monitor_->setEnvironmentPublishingFrequency(10.0);
    monitor_->startPublishingEnvironment();

    // Create and configure the plotter to visualize the environment (and other things)
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName(), "samxl");
  }

protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
}
