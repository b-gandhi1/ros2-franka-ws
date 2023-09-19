#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  std::vector<std::string> joint_names = move_group.getJointNames();
  //   std::vector<double> joint_values = { 1.923,-1.723, -1.4935, -1.945, -2.178, 1.996, 1.562 };

  std::vector<double> joint_values;

  while (rclcpp::ok())
  {
    joint_values = { 1.663, -1.735, -1.542, -2.328, -2.276, 1.909, 1.689 };
    move_group.setJointValueTarget(joint_names, joint_values);
    move_group.setMaxVelocityScalingFactor(0.1);
    move_group.move();

    joint_values = { 1.418, -1.748, -1.680, -2.294, -2.138, 2.380, 1.695 };
    move_group.setJointValueTarget(joint_names, joint_values);
    move_group.move();

    joint_values = { 1.663, -1.735, -1.542, -2.328, -2.276, 1.909, 1.689 };
    move_group.setJointValueTarget(joint_names, joint_values);
    move_group.move();

    joint_values = { 1.869, -1.716, -1.531, -2.386, -2.457, 1.562, 1.582 };
    move_group.setJointValueTarget(joint_names, joint_values);
    move_group.move();
  }

  rclcpp::shutdown();
  return 0;
}