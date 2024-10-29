#include <rclcpp/node.hpp>

// For the robot_description publisher
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <chrono>

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_kinematics/core/kinematic_group.h>

static const std::string COLLISION_LINK_NAME = "dynamic_collision_object";

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

    // Create a publisher for the robot description (for the joint state publisher, etc.)
    urdf_pub_ = create_publisher<std_msgs::msg::String>("robot_description", rclcpp::QoS(1).reliable().transient_local());
    {
      std_msgs::msg::String msg;
      msg.data = urdf;
      urdf_pub_->publish(msg);
    }

    monitor_ = std::make_shared<tesseract_monitoring::ROSEnvironmentMonitor>(node_ptr, env_, "samxl");
    monitor_->startStateMonitor("/joint_states", true);
    monitor_->setEnvironmentPublishingFrequency(30.0);
    monitor_->startPublishingEnvironment();

    // Create and configure the plotter to visualize the environment (and other things)
    plotter_ = std::make_shared<tesseract_rosutils::ROSPlotting>(env_->getRootLinkName(), "samxl");

    // Add a link to the environment to represent a dynamic collision obstacle
    {
      tesseract_scene_graph::Link link(COLLISION_LINK_NAME);

      // Add some visual geometry to the link
      const double radius = 0.5; // meters
      const double length = 2.0; // meters
      auto cylinder = std::make_shared<const tesseract_geometry::Cylinder>(radius, length);

      auto visual = std::make_shared<tesseract_scene_graph::Visual>();
      visual->geometry = cylinder;
      visual->origin.translate(Eigen::Vector3d(0.0, 0.0, length / 2.0));
      link.visual.push_back(visual);

      auto collision = std::make_shared<tesseract_scene_graph::Collision>();
      collision->geometry = cylinder;
      collision->origin = visual->origin;
      link.collision.push_back(collision);

      // Create a joint to connect this new link to the world (root) link
      tesseract_scene_graph::Joint joint("dynamic_collision_object_joint");
      joint.parent_link_name = env_->getRootLinkName();
      joint.child_link_name = link.getName();
      joint.type = tesseract_scene_graph::JointType::FIXED;

      auto cmd = std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, false);
      if (!env_->applyCommand(cmd))
        throw std::runtime_error("Failed to add dynamic collision link");
    }

    // Make a timer to move an arbitrary link around in the environment
    server_ = create_service<std_srvs::srv::Trigger>("move_object",
                                           std::bind(&PlanningServer::moveCylinderLinkCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void moveCylinderLinkCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    // Specify the type of the joint
    tesseract_scene_graph::Joint joint("new_dynamic_collision_object_joint");
    joint.type = tesseract_scene_graph::JointType::FIXED;

    // Extract the name of the TCP from the "gantry_7dof" group (defined in the SRDF)
    // tesseract_kinematics::KinematicGroup::Ptr kin_group = env_->getKinematicGroup("gantry_7dof");
    // if (!kin_group)
    // {
    //   res->message = "No kinematic group found";
    //   res->success = false;
    //   return;
    // }

    // std::vector<std::string> tip_links = kin_group->getAllPossibleTipLinkNames();
    // if (tip_links.empty())
    // {
    //   res->message = "No tip links found for planning group 'manipulator'";
    //   res->success = false;
    //   return;
    // }

    static bool on = true;
    if(on)
    {
      joint.parent_link_name = "gantry_tool0";
      joint.child_link_name = COLLISION_LINK_NAME;
    }
    else
    {
      joint.parent_link_name = env_->getRootLinkName();
      joint.child_link_name = COLLISION_LINK_NAME;
    }
    on = !on;

    // Apply the command
    auto cmd = std::make_shared<tesseract_environment::MoveLinkCommand>(joint);
    if (!env_->applyCommand(cmd))
    {
      res->message = "Failed to move collision object to new TCP link";
      res->success = false;
      return;
    }

    res->success = true;
  }

protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
}
