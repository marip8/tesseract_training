#include <rclcpp/node.hpp>
#include <rclcpp/duration.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <console_bridge/console.h>
#include <chrono>

// Tesseract environment includes
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_environment/environment.h>

// Tesseract scene graph includes
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>

// Tesseract command language includes
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/utils.h>
#include <tesseract_command_language/profile_dictionary.h>

// Tesseract task composer includes
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>

// Tesseract ROS Utils
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>

// Tesseract includes (misc.)
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_visualization/markers/axis_marker.h>

// =============================================================================
// SAM XL Gantry Cell Settings
// =============================================================================

static const std::string GROUP_NAME = "manipulator"; // Assuming "penelope_tool_cell"
static const std::string TCP = "offset_75cm_inv"; // offset of 75cm from flange, Z pointing from tcp location to flange.

static const std::string WOBJ_1 = "plate_6"; // bottem left plate, top right corner
static const std::string WOBJ_2 = "plate_5"; // middle plate (with stringers), top right corner
static const std::string WOBJ_3 = "plate_1"; // top right plate, top right corner

// =============================================================================
// Planning Settings
// =============================================================================

static const std::string PROFILE = "DEFAULT_SAMXL";


// =============================================================================
// ROS Parameter Settings
// =============================================================================

static const std::string TASK_COMPOSER_CONFIG_FILE_PARAM = "task_composer_file";
static const std::string TASK_NAME_PARAM = "task_name";


class PlanningServer : public rclcpp::Node
{
public:
  PlanningServer()
    : rclcpp::Node("gantry_planning_problem_1")
    , env_(std::make_shared<tesseract_environment::Environment>())
  {

    // Update log level for debugging
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

    // Get the URDF and SRDF from parameter
    declare_parameter("robot_description", "");
    declare_parameter("robot_description_semantic", "");
    const std::string urdf = get_parameter("robot_description").as_string();
    const std::string srdf = get_parameter("robot_description_semantic").as_string();

    // Declare a parameter for the task composer file
    declare_parameter(TASK_COMPOSER_CONFIG_FILE_PARAM, "");
    declare_parameter(TASK_NAME_PARAM, "");

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
    trajectory_action_client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(this, "follow_joint_trajectory");

    // Create a program to get it to publish
  }

  void planCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    try
    {
      auto program = PlanningServer::createProgram();
      auto trajectory = PlanningServer::planTrajectory(program);
    }
    catch(const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
      return;
    }

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

  tesseract_common::JointTrajectory plotTrajectory(const std::string& key,
                                                   tesseract_planning::TaskComposerContext::ConstPtr context)
  {
    // Get results of successful plan
    tesseract_planning::CompositeInstruction program_results =
        context->data_storage->getData(key).as<tesseract_planning::CompositeInstruction>();

    // Send joint trajectory to Tesseract plotter widget
    tesseract_common::JointTrajectory jt = toJointTrajectory(program_results);
    plotter_->plotTrajectory(jt, *env_->getStateSolver());

    return jt;
  }

  // tesseract_planning::CompositeInstruction createProgram2()
  // {
  //   using namespace tesseract_planning;

  //   auto default_mi = tesseract_common::ManipulatorInfo();
  //   default_mi.manipulator = GROUP_NAME;
  //   default_mi.working_frame = "gantry_base";
  //   default_mi.tcp_frame = TCP;

  //   auto program = tesseract_planning::CompositeInstruction(PROFILE, tesseract_planning::CompositeInstructionOrder::ORDERED, default_mi);

  //   // Add the current state of the robot as the start state of the trajectory
  //   const std::vector<std::string> joint_names = env_->getJointGroup(default_mi.manipulator)->getJointNames();
  //   StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));
  //   program.appendMoveInstruction(
  //       MoveInstruction(StateWaypointPoly{ current_state }, MoveInstructionType::FREESPACE, PROFILE, default_mi));

  //   // Waypoint 1
  //   Eigen::Isometry3d w1 = Eigen::Isometry3d::Identity();
  //   w1.translate(Eigen::Vector3d(3.0, 3.0, 2.0));

  //   program.appendMoveInstruction(
  //       MoveInstruction(CartesianWaypointPoly{ CartesianWaypoint(w1) }, MoveInstructionType::LINEAR, PROFILE, default_mi));

  //   // Waypoint 2
  //   Eigen::Isometry3d w2 = Eigen::Isometry3d::Identity();
  //   w2.translate(Eigen::Vector3d(5.0, 5.0, 2.0));

  //   program.appendMoveInstruction(
  //       MoveInstruction(CartesianWaypointPoly{ CartesianWaypoint(w2) }, MoveInstructionType::LINEAR, PROFILE, default_mi));

  //   return program;
  // }

  tesseract_planning::CompositeInstruction createProgram()
  {
    using namespace tesseract_planning;

    auto default_mi = tesseract_common::ManipulatorInfo();
    default_mi.manipulator = GROUP_NAME;
    default_mi.working_frame = "gantry_base";
    default_mi.tcp_frame = TCP;

    // Add the current state of the robot as the start state of the trajectory
    const std::vector<std::string> joint_names = env_->getJointGroup(GROUP_NAME)->getJointNames();
    StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));

    // Poses used for the plate movements
    const tesseract_common::JointState seed(joint_names, env_->getCurrentJointValues(joint_names));

    const double z_offset = 0.35;

    auto plate_wp1 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, z_offset)));
    plate_wp1.setSeed(seed);

    auto plate_wp2 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.25, 0.0, z_offset)));
    plate_wp2.setSeed(seed);

    auto plate_wp3 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.25, z_offset)));
    plate_wp3.setSeed(seed);

    auto plate_wp4 = tesseract_planning::CartesianWaypoint(Eigen::Isometry3d(Eigen::Translation3d(0.25, 0.25, z_offset)));
    plate_wp4.setSeed(seed);

    // Define manipulator groups for each plate
    auto mi_wobj_1 = tesseract_common::ManipulatorInfo();
    mi_wobj_1.manipulator = GROUP_NAME;
    mi_wobj_1.working_frame = WOBJ_1;
    mi_wobj_1.tcp_frame = TCP;

    auto mi_wobj_3 = tesseract_common::ManipulatorInfo();
    mi_wobj_3.manipulator = GROUP_NAME;
    mi_wobj_3.working_frame = WOBJ_3;
    mi_wobj_3.tcp_frame = TCP;

    // Create the top-level container
    auto program = tesseract_planning::CompositeInstruction(PROFILE, CompositeInstructionOrder::ORDERED, default_mi);

    // Define from the start waypoint to the first raster waypoint
    {
      tesseract_planning::CompositeInstruction from_start(PROFILE, CompositeInstructionOrder::ORDERED, mi_wobj_1);
      from_start.appendMoveInstruction(
          MoveInstruction(StateWaypointPoly{ current_state }, MoveInstructionType::FREESPACE, PROFILE, mi_wobj_1));

      from_start.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp1 }, MoveInstructionType::FREESPACE, PROFILE, mi_wobj_1));

      program.push_back(from_start);
    }

    // WP1: Move to "jig_entry"
    // auto wp1 = getWaypointFromGroupState(GROUP_NAME, "jig_entry");
    // program.appendMoveInstruction(
    //   MoveInstruction(JointWaypointPoly{ wp1 }, MoveInstructionType::LINEAR, PROFILE, default_mi
    // ));

    // WP2: Move to "jig_middle"
    // auto wp2 = getWaypointFromGroupState(GROUP_NAME, "jig_middle");
    // program.appendMoveInstruction(
    //   MoveInstruction(JointWaypointPoly{ wp2 }, MoveInstructionType::LINEAR, PROFILE, default_mi
    // ));

    // WP3-6: Move over WOBJ_1
    {
      tesseract_planning::CompositeInstruction raster_1(PROFILE, CompositeInstructionOrder::ORDERED, mi_wobj_1);

      // raster_1.appendMoveInstruction(
      //     MoveInstruction(CartesianWaypointPoly{ plate_wp1 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_1)
      //     );
      raster_1.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp2 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_1)
          );
      raster_1.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp3 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_1)
          );
      raster_1.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp4 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_1)
          );

      program.push_back(raster_1);
    }

    // Add a transition
    {
      tesseract_planning::CompositeInstruction transition(PROFILE, CompositeInstructionOrder::ORDERED, mi_wobj_3);

      transition.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp1 }, MoveInstructionType::FREESPACE, PROFILE, mi_wobj_3)
          );

      program.push_back(transition);
    }

    // WP7-10:: Move over WOBJ_3
    {
      tesseract_planning::CompositeInstruction raster_2(PROFILE, CompositeInstructionOrder::ORDERED, mi_wobj_3);

      // raster_2.appendMoveInstruction(
      //     MoveInstruction(CartesianWaypointPoly{ plate_wp1 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_3)
      //     );
      raster_2.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp2 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_3)
          );
      raster_2.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp3 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_3)
          );
      raster_2.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{ plate_wp4 }, MoveInstructionType::LINEAR, PROFILE, mi_wobj_3)
          );

      program.push_back(raster_2);
    }

    // WP11: Move to "home"
    // auto wp11 = getWaypointFromGroupState(GROUP_NAME, "home");
    // program.appendMoveInstruction(
    //   MoveInstruction(JointWaypointPoly{ wp1 }, MoveInstructionType::FREESPACE, "DEFAULT", default_mi
    // ));

    // Define a move back to the current robot state
    {
      tesseract_planning::CompositeInstruction to_home(PROFILE, CompositeInstructionOrder::ORDERED, mi_wobj_1);

      to_home.appendMoveInstruction(
          MoveInstruction(StateWaypointPoly{ current_state }, MoveInstructionType::FREESPACE, PROFILE, mi_wobj_1));

      program.push_back(to_home);
    }

    // Plot the waypoints
    {
      const Eigen::Isometry3d root_to_plate_1 = env_->getLinkTransform(WOBJ_1);
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_1 * plate_wp1.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_1 * plate_wp2.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_1 * plate_wp3.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_1 * plate_wp4.getTransform()));

      const Eigen::Isometry3d root_to_plate_3 = env_->getLinkTransform(WOBJ_3);
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_3 * plate_wp1.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_3 * plate_wp2.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_3 * plate_wp3.getTransform()));
      plotter_->plotMarker(tesseract_visualization::AxisMarker(root_to_plate_3 * plate_wp4.getTransform()));
    }

    return program;
  }

  tesseract_planning::ProfileDictionary::Ptr createProfileDict()
  {
    using namespace tesseract_planning;

    auto profile_dict = std::make_shared<ProfileDictionary>();
    
    // TODO: Add profiles
    auto profile = std::make_shared<tesseract_planning::SimplePlannerLVSPlanProfile>();
    profile_dict->addProfile<tesseract_planning::SimplePlannerPlanProfile>("SimplePlanner", PROFILE, profile);

    return profile_dict;
  }

  trajectory_msgs::msg::JointTrajectory planTrajectory(const tesseract_planning::CompositeInstruction& program)
  {
    // Set up task composer problem
    auto task_composer_config_file = get_parameter(TASK_COMPOSER_CONFIG_FILE_PARAM).as_string();
    const YAML::Node task_composer_config = YAML::LoadFile(task_composer_config_file);
    tesseract_planning::TaskComposerPluginFactory factory(task_composer_config);

    auto task_name = get_parameter(TASK_NAME_PARAM).as_string();
    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
    tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
    if (!task)
      throw std::runtime_error("Failed to create '" + task_name + "' task");

    // Save dot graph
    {
      std::ofstream tc_out_data(tesseract_common::getTempPath() + task_name + ".dot");
      task->dump(tc_out_data);
    }
    
    //
    const std::string input_key = task->getInputKeys().get("program");
    const std::string output_key = task->getOutputKeys().get("trajectory");

    auto task_data = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
    task_data->setData(input_key, program);
    task_data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    task_data->setData("profiles", createProfileDict());

    // Dump dotgraphs of each task for reference
    {
      const YAML::Node& task_plugins = task_composer_config["task_composer_plugins"]["tasks"]["plugins"];
      for (auto it = task_plugins.begin(); it != task_plugins.end(); ++it)
      {
        auto task_plugin_name = it->first.as<std::string>();
        std::ofstream f(tesseract_common::getTempPath() + task_plugin_name + ".dot");
        tesseract_planning::TaskComposerNode::Ptr task = factory.createTaskComposerNode(task_plugin_name);
        if (!task)
          throw std::runtime_error("Failed to load task: '" + task_plugin_name + "'");
        task->dump(f);
      }
    }

    // Run problem
    tesseract_planning::TaskComposerFuture::UPtr result = executor->run(*task, task_data, true);
    result->wait();

    // Save the output dot graph
    {
      std::ofstream tc_out_results(tesseract_common::getTempPath() + task_name + "_results.dot");
      static_cast<const tesseract_planning::TaskComposerGraph&>(*task).dump(tc_out_results, nullptr,
                                                                            result->context->task_infos.getInfoMap());
    }

    // Plot the results of the intermediate planner steps
    try
    {
      plotTrajectory("simple_planner_trajectory", result->context);
    }
    catch(const std::exception&)
    {
    }

    try
    {
      plotTrajectory("descartes_trajectory", result->context);
    }
    catch(const std::exception&)
    {
    }

    // Check for successful plan
    if (!result->context->isSuccessful() || result->context->isAborted())
      throw std::runtime_error("Failed to create motion plan");

    // Get results of successful plan
    tesseract_common::JointTrajectory jt  = plotTrajectory(output_key, result->context);

    return tesseract_rosutils::toMsg(jt, env_->getState());
  }

  void executeTrajectory(trajectory_msgs::msg::JointTrajectory trajectory)
  {
    // TODO: Wait for action server

    auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
    goal_msg.trajectory = trajectory;
    goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(5.0);

    // TODO: Send action goal
  }


protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_server_;
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr trajectory_action_client_ptr_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
}
