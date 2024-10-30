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

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>

static const std::string GROUP_NAME = "manipulator"; // Assuming "penelope_tool_cell"
static const std::string TCP = "offset_75cm_inv"; // offset of 75cm from flange, Z pointing from tcp location to flange.

static const std::string WOBJ_1 = "plate_6"; // bottem left plate, top right corner
static const std::string WOBJ_2 = "plate_5"; // middle plate (with stringers), top right corner
static const std::string WOBJ_3 = "plate_1"; // top right plate, top right corner

class PlanningServer : public rclcpp::Node
{
public:
  PlanningServer()
    : rclcpp::Node("gantry_planning_problem_1")
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

    // Create ROS Service to plan trajectory
    plan_server_ = create_service<std_srvs::srv::Trigger>("plan_trajectory", std::bind(&PlanningServer::planCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void planCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    auto program = PlanningServer::createProgram();
    auto trajectory = PlanningServer::planTrajectory(program);

    // TODO: Visualize trajectory

    res->success = true;
  }
  
  tesseract_planning::JointWaypoint getWaypointFromGroupState(std::string group_name, std::string state_name)
  {
    auto kin_info = env_->getKinematicsInformation();
    
    // Check if state exists
    if (!kin_info.hasGroup(group_name))
        throw std::runtime_error("Unknown group");
    if (!kin_info.hasGroupJointState(group_name, state_name))
        throw std::runtime_error("Unkown state");

    auto waypoint = tesseract_planning::JointWaypoint();
    // TODO: fill in waypoint based on GroupState from SRDF

    // TODO: Reorder JointWaypoint (needed as order might be wrong when retrieved from SRDF)

    return waypoint;
  }

  tesseract_planning::CompositeInstruction createProgram()
  {
    auto default_mi = tesseract_common::ManipulatorInfo();
    // TODO: Populate the default ManipulatorInfo

    auto program = tesseract_planning::CompositeInstruction("DEFAULT", tesseract_planning::CompositeInstructionOrder::ORDERED, default_mi);


    // WP1: Move to "jig_entry"
    auto wp1 = getWaypointFromGroupState(GROUP_NAME, "jig_entry");
    // TODO: Add waypoint to program


    // WP2: Move to "jig_middle"
    auto wp2 = getWaypointFromGroupState(GROUP_NAME, "jig_middle");
    // TODO: Add waypoint to program


    // Poses used for the plate movements
    auto plate_wp1 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0)));
    auto plate_wp2 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.25, 0.0, 0.0)));
    auto plate_wp3 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.25, 0.0)));
    auto plate_wp4 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.25, 0.25, 0.0)));


    // WPXX-XX: Move over plate 6
    auto mi_plate_6 = tesseract_common::ManipulatorInfo();
    // TODO: Populate the ManipulatorInfo for "plate_6"

    auto ci_plate_6 = tesseract_planning::CompositeInstruction("DEFAULT", tesseract_planning::CompositeInstructionOrder::ORDERED, mi_plate_6);
    // TODO: Add waypoints to ci_plate_6
    // TODO: Add ci_plate_6 to program


    // WPXX-XX:: Move over plate 1
    auto mi_plate_1 = tesseract_common::ManipulatorInfo();
    // TODO: Populate the ManipulatorInfo for "plate_1"

    auto ci_plate_1 = tesseract_planning::CompositeInstruction("DEFAULT", tesseract_planning::CompositeInstructionOrder::ORDERED, mi_plate_1);
    // TODO: Add waypoints to ci_plate_1
    // TODO: Add ci_plate_1 to program


    // WPXX: Move to "home"
    auto wp10 = getWaypointFromGroupState(GROUP_NAME, "home");
    // TODO: Add waypoint to program

    return program;
  }

  tesseract_planning::CompositeInstruction planTrajectory(tesseract_planning::CompositeInstruction program)
  {
    // TODO: Plan the trajectory using Tesseract

    return program;
  }

  void executeTrajectory(tesseract_planning::CompositeInstruction trajectory)
  {
    // TODO: Send the trajectory to the Gantry action interface
  }


protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
}
