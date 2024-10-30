#include <rclcpp/node.hpp>

// For the robot_description publisher
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/string.hpp>

#include <console_bridge/console.h>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>

#include <tesseract_environment/environment.h>
#include <tesseract_monitoring/environment_monitor.h>
#include <tesseract_rosutils/plotting.h>
#include <tesseract_rosutils/utils.h>
#include <tesseract_environment/commands/add_link_command.h>
#include <tesseract_environment/commands/move_link_command.h>
#include <tesseract_environment/commands/change_joint_origin_command.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_state_solver/state_solver.h>

// Tesseract command language includes
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/utils.h>

// Tesseract task composer includes
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_graph.h>

static const std::string COLLISION_LINK_NAME = "dynamic_collision_object";
static const std::string COLLISION_JOINT_NAME = "dynamic_collision_object_joint";
static const std::string PROFILE = "samxl";
static const std::string TASK_COMPOSER_CONFIG_FILE_PARAM = "task_composer_file";

// Add a link to the environment to represent a dynamic collision obstacle
tesseract_environment::Command::Ptr createAddLinkCommand(const std::string& parent_link)
{
  tesseract_scene_graph::Link link(COLLISION_LINK_NAME);

  // Add some visual geometry to the link
  const double radius = 0.5; // meters
  const double length = 2.0; // meters
  auto cylinder = std::make_shared<tesseract_geometry::Cylinder>(radius, length);

  auto visual = std::make_shared<tesseract_scene_graph::Visual>();
  visual->geometry = cylinder;
  visual->origin.translate(Eigen::Vector3d(0.0, 0.0, length / 2.0));
  link.visual.push_back(visual);

  auto collision = std::make_shared<tesseract_scene_graph::Collision>();
  collision->geometry = cylinder;
  collision->origin = visual->origin;
  link.collision.push_back(collision);

  // Create a joint to connect this new link to the world (root) link
  tesseract_scene_graph::Joint joint(COLLISION_JOINT_NAME);
  joint.parent_link_name = parent_link;
  joint.child_link_name = link.getName();
  joint.type = tesseract_scene_graph::JointType::FIXED;

  return std::make_shared<tesseract_environment::AddLinkCommand>(link, joint, false);
}

class PlanningServer : public rclcpp::Node
{
public:
  PlanningServer()
    : rclcpp::Node("planning_server")
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

    // Add a link to the environment
    // env_->applyCommand(createAddLinkCommand(env_->getRootLinkName()));

    // Make a timer to move an arbitrary link around in the environment
    // server_ = create_service<std_srvs::srv::Trigger>("move_object",
                                                     // std::bind(&PlanningServer::moveCylinderLinkCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Create a timer to move the object around dynamically
    // timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PlanningServer::timerCallback, this));

    // Create a server for motion planning
    plan_server_ = create_service<std_srvs::srv::Trigger>("plan", std::bind(&PlanningServer::planCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  tesseract_planning::CompositeInstruction createProgram(const tesseract_common::ManipulatorInfo& info,
                                                         const tesseract_common::VectorIsometry3d& poses)
  {
    using namespace tesseract_planning;

    CompositeInstruction program(PROFILE, CompositeInstructionOrder::ORDERED, info);

    // Get the names of the joints associated with the input manipulator group
    const std::vector<std::string> joint_names = env_->getJointGroup(info.manipulator)->getJointNames();

    // Add the current state of the robot as the start state of the trajectory
    {
      StateWaypoint current_state(joint_names, env_->getCurrentJointValues(joint_names));

      // Define a "move" to the start waypoint
      program.appendMoveInstruction(
          MoveInstruction(StateWaypointPoly{ current_state }, MoveInstructionType::FREESPACE, PROFILE, info));
    }

    // Add move instructions to the input poses
    for(const Eigen::Isometry3d& pose : poses)
    {
      CartesianWaypoint waypoint(pose);
      program.appendMoveInstruction(
          MoveInstruction(CartesianWaypointPoly{waypoint}, MoveInstructionType::LINEAR, PROFILE, info));
    }

    return program;
  }

  void timerCallback()
  {
    const static double start_time = get_clock()->now().seconds();
    double t = get_clock()->now().seconds() - start_time;

    const double r = 0.25;
    double x = r * std::cos(t);
    double y = r * std::sin(t);

    Eigen::Isometry3d new_origin = Eigen::Isometry3d::Identity();
    new_origin.translate(Eigen::Vector3d(x, y, 0.0));
    auto cmd = std::make_shared<tesseract_environment::ChangeJointOriginCommand>(COLLISION_JOINT_NAME, new_origin);

    RCLCPP_INFO(get_logger(), "Moving object");
    if (!env_->applyCommand(cmd))
      RCLCPP_ERROR(get_logger(), "Failed to move the joint");
  }

  void moveCylinderLinkCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    // Specify the type of the joint
    tesseract_scene_graph::Joint joint(COLLISION_JOINT_NAME);
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
      joint.parent_link_name = "wrist_tool0";
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

  trajectory_msgs::msg::JointTrajectory plan(const tesseract_planning::CompositeInstruction& program,
                                                // tesseract_planning::ProfileDictionary::Ptr profile_dict,
                                                const std::string& task_name)
  {
    // Set up task composer problem
    auto task_composer_config_file = get_parameter(TASK_COMPOSER_CONFIG_FILE_PARAM).as_string();
    const YAML::Node task_composer_config = YAML::LoadFile(task_composer_config_file);
    tesseract_planning::TaskComposerPluginFactory factory(task_composer_config);

    auto executor = factory.createTaskComposerExecutor("TaskflowExecutor");
    tesseract_planning::TaskComposerNode::UPtr task = factory.createTaskComposerNode(task_name);
    if (!task)
      throw std::runtime_error("Failed to create '" + task_name + "' task");

    // Save dot graph
    {
      std::ofstream tc_out_data(tesseract_common::getTempPath() + task_name + ".dot");
      task->dump(tc_out_data);
    }

    const std::string input_key = task->getInputKeys().get("program");
    const std::string output_key = task->getOutputKeys().get("program");
    auto task_data = std::make_shared<tesseract_planning::TaskComposerDataStorage>();
    task_data->setData(input_key, program);
    task_data->setData("environment", std::shared_ptr<const tesseract_environment::Environment>(env_));
    // task_data->setData("profiles", profile_dict);

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

    // Check for successful plan
    if (!result->context->isSuccessful() || result->context->isAborted())
      throw std::runtime_error("Failed to create motion plan");

    // Get results of successful plan
    tesseract_planning::CompositeInstruction program_results =
        result->context->data_storage->getData(output_key).as<tesseract_planning::CompositeInstruction>();

    // Send joint trajectory to Tesseract plotter widget
    tesseract_common::JointTrajectory jt = toJointTrajectory(program_results);
    plotter_->plotTrajectory(jt, *env_->getStateSolver());

    return tesseract_rosutils::toMsg(jt, env_->getState());
  }

  void planCallback(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                    std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    try
    {
      tesseract_common::ManipulatorInfo info("mobile_manipulator", "map", "ur10e_tool0");

      Eigen::Isometry3d pose_1 = Eigen::Isometry3d::Identity();
      pose_1.translate(Eigen::Vector3d(3.0, 3.0, 1.0)).rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

      tesseract_common::VectorIsometry3d poses {
        pose_1,
      };

      tesseract_planning::CompositeInstruction program = createProgram(info, poses);

      trajectory_msgs::msg::JointTrajectory trajectory = plan(program, "");

      // Forward to JTA to execute trajectory
      // ...

      res->success = true;
    }
    catch(const std::exception& ex)
    {
      res->message = ex.what();
      res->success = false;
    }
  }

protected:
  tesseract_environment::Environment::Ptr env_;
  tesseract_monitoring::ROSEnvironmentMonitor::Ptr monitor_;
  tesseract_rosutils::ROSPlotting::Ptr plotter_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr urdf_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_server_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<PlanningServer>();
  rclcpp::spin(server);
}
