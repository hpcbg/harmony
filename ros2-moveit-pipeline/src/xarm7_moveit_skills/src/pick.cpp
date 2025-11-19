#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_simple");
namespace mtc = moveit::task_constructor;

class PickNode
{
public:
  PickNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();

  void setupPlanningScene();

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr PickNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

PickNode::PickNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("pick_node", options) }
{
}

void PickNode::setupPlanningScene()
{

  const std::string planning_frame = "world";

  moveit_msgs::msg::CollisionObject object;
  object.id = "bottle";
  object.header.frame_id = planning_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };  // [height, radius]

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.57957;
  pose.position.y = -0.08264;
  pose.position.z = 0.12218;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  
  RCLCPP_INFO(LOGGER, "Added bottle to planning scene at [%.3f, %.3f, %.3f]",
              pose.position.x, pose.position.y, pose.position.z);

  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = planning_frame;
  collision_object.id = "table";
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 2.474;
  primitive.dimensions[primitive.BOX_Y] = 0.762;
  primitive.dimensions[primitive.BOX_Z] = 0.994;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.x = 0.0;
  box_pose.orientation.y = 0.0;
  box_pose.orientation.z = 0.0;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 1.05589;
  box_pose.position.y = 0.0;
  box_pose.position.z = -0.497; // shift it up by half of the height

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  psi.applyCollisionObject(collision_object);

  RCLCPP_INFO(LOGGER, "Added table to planning scene at [%.3f, %.3f, %.3f]",
              box_pose.position.x, box_pose.position.y, box_pose.position.z);
}

void PickNode::doTask()
{
  task_ = createTask();

  try
  {
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  
  RCLCPP_INFO(LOGGER, "Task planning succeeded! Found %zu solutions", 
              task_.solutions().size());
  
  task_.introspection().publishSolution(*task_.solutions().front());

  // Uncomment to execute
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
     RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
     return;
  }
  RCLCPP_INFO(LOGGER, "Task execution succeeded!");

  return;
}

mtc::Task PickNode::createTask()
{
  
  mtc::Task task;
  
  task.stages()->setName("simple pick test");
  task.loadRobotModel(node_);

  const std::string planning_frame = task.getRobotModel()->getModelFrame();
  // Log the planning frame
  RCLCPP_INFO(LOGGER, "Planning frame: %s", planning_frame.c_str());

  const auto& arm_group_name = "xarm7_arm";
  const auto& hand_group_name = "xarm7_hand";
  const auto& hand_frame = "link_tcp";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  // Create planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  sampling_planner->setPlannerId("RRTConnectkConfigDefault");
  
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  // 1. Start from current state
  auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
  task.add(std::move(current_state));

  // 2. Open the gripper
  auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
  open_gripper->setGroup(hand_group_name);
  open_gripper->setGoal("open");
  task.add(std::move(open_gripper));

  // 3. Move to ready pose
  auto stage_move_to_ready =
      std::make_unique<mtc::stages::MoveTo>("move to ready", sampling_planner);
  stage_move_to_ready->setGroup(arm_group_name);
  stage_move_to_ready->setGoal("ready"); // Assumes "ready" is in your SRDF
  task.add(std::move(stage_move_to_ready));

  // 3. Move arm to approach pose (above the bottle)
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("move above bottle", sampling_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGroup(arm_group_name);
    
    // Define pose above the bottle
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = planning_frame;
    
    // Position above bottle (15cm higher)
    // target_pose.pose.position.x = -0.47957;
    // target_pose.pose.position.y = -0.08264;
    // target_pose.pose.position.z = 1.09218 + 0.15;  // 15cm above bottle

    target_pose.pose.position.x = 0.57957;
    target_pose.pose.position.y = -0.08264;
    target_pose.pose.position.z = 0.12218 + 0.15;  // 15cm above bottle
    
    // Gripper pointing down
    target_pose.pose.orientation.x = 1.0;
    target_pose.pose.orientation.y = 0.0;
    target_pose.pose.orientation.z = 0.0;
    target_pose.pose.orientation.w = 0.0;
    
    stage->setGoal(target_pose);
    task.add(std::move(stage));
  }

  RCLCPP_INFO(LOGGER, "Created simple task with %zu stages", task.stages()->numChildren());
  
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<PickNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });

  // Give time for everything to initialize
  std::this_thread::sleep_for(std::chrono::seconds(2));

  mtc_task_node->setupPlanningScene();
  
  // Give time for planning scene to update
  std::this_thread::sleep_for(std::chrono::seconds(1));
  
  mtc_task_node->doTask();

  // Keep spinning for visualization
  RCLCPP_INFO(LOGGER, "Task complete. Press Ctrl+C to exit.");
  spin_thread->join();
  
  rclcpp::shutdown();
  return 0;
}