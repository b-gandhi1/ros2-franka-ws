#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <tuple>

// #include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

std::vector<double> joint_values1, joint_values2, joint_values3, joint_values4;

std::tuple<std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>> transZ2() // drawer new pos as labelled
{
  joint_values1 = {1.2215, -1.7602, -1.8127, -2.6935, -2.4901, 2.0506, 1.9047}; // Tz, DOWN
  joint_values2 = {1.2426, -1.7615, -1.7752, -2.6919, -2.4811, 2.0373, 1.9152}; // Tz, MIDDLE
  joint_values3 = {1.2554, -1.7619, -1.7495, -2.6890, -2.4798, 2.0341, 1.9151}; // Tz, UP
  joint_values4 = {1.2426, -1.7615, -1.7752, -2.6919, -2.4811, 2.0373, 1.9152}; // Tz, MIDDLE
  
  return std::make_tuple(joint_values1, joint_values2, joint_values3, joint_values4);
}

std::tuple<std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>> rotY1() // drawer extended all the way out
{
  joint_values1 = {1.711,-1.749,-1.421,-2.283,-2.061,1.828,1.444}; // middle position 1
  joint_values2 = {1.494,-1.756,-1.540,-2.320,-1.999,2.153,1.497}; // towards me 1
  joint_values3 = {1.711,-1.749,-1.421,-2.283,-2.061,1.828,1.444}; // same middle again 1
  joint_values4 = {1.934,-1.732,-1.414,-2.321,-2.308,1.484,1.410}; // away from me 1
  
  return std::make_tuple(joint_values1, joint_values2, joint_values3, joint_values4);
}

std::tuple<std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>> rotY2() // drawer new pos as labelled
{
  joint_values1 = {1.2549,-1.7506,-1.7255,-2.7260,-2.3606,1.9730,1.7437}; // middle pos 2
  joint_values2 = {1.1017,-1.7605,-1.8026,-2.7145,-2.3314,2.1215,1.8110}; // towards me 2
  joint_values3 = {1.2549,-1.7506,-1.7255,-2.7260,-2.3606,1.9730,1.7437}; // same middle again 2
  joint_values4 = {1.4685,-1.7490,-1.7105,-2.7661,-2.5014,1.6631,1.7115}; // away from me 2

  return std::make_tuple(joint_values1, joint_values2, joint_values3, joint_values4);
}

std::tuple<std::vector<double>,std::vector<double>,std::vector<double>,std::vector<double>> rotY_newman()
{
  joint_values1={1.3965, -1.7091, -1.7673, -2.5708, -2.5066, 2.1008, 1.7940}; // middle
  joint_values2={1.6556, -1.7078, -1.7361, -2.6458, -2.5922, 1.6585, 1.6732}; // towards me
  joint_values3=joint_values1; // middle
  joint_values4={1.1734, -1.7355, -1.8791, -2.5309, -2.4114, 2.4029, 1.9135}; // away from me
  
  return std::make_tuple(joint_values1, joint_values2, joint_values3, joint_values4);
}
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options);

  // Publisher
  auto pub_ = move_group_node->create_publisher<std_msgs::msg::Int32>("camera_trigger", 10);
  auto pub_2 = move_group_node->create_publisher<geometry_msgs::msg::PoseStamped>("franka_ee_pos", 10);

  rclcpp::WallRate loop_rate(0.25);
  rclcpp::WallRate loop_rate2(1);
  // We spin up a SingleThreadedExecutor for the current state monitor to get information
  // about the robot's state.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]()
              { executor.spin(); })
      .detach();

  static const std::string PLANNING_GROUP = "panda_arm";

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);

  std::vector<std::string> joint_names = move_group.getJointNames();

  // std::vector<double> joint_values1;
  std::vector<double> joint_values_1, joint_values_2, joint_values_3, joint_values_4;
  std_msgs::msg::Int32 msg;
  int count = 0;

  msg.data = 0;
  pub_->publish(msg);
  loop_rate.sleep();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success;

  while (rclcpp::ok())
  {
    // msg.data = 1;
    // pub_->publish(msg);
    if (count == 0)
    {
      msg.data = 1;
      pub_->publish(msg);
      count++;
    }
    else
    {
      tie(joint_values_1, joint_values_2, joint_values_3, joint_values_4) =  rotY_newman(); // is there a way to pass this as an argument with ros2 run?? 

      move_group.setJointValueTarget(joint_names, joint_values_1);
      move_group.setMaxVelocityScalingFactor(0.1);
      success = (move_group.plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      move_group.execute(my_plan);
      // move_group.move();
      // sleep(2); // for 2 seconds, for Tz motion
      // loop_rate2.sleep();

      move_group.setJointValueTarget(joint_names, joint_values_2);
      success = (move_group.plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      move_group.execute(my_plan);
      // move_group.move();

      // sleep(2); // for 2 seconds, for Tz motion

      // loop_rate2.sleep();

      move_group.setJointValueTarget(joint_names, joint_values_3);
      success = (move_group.plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      move_group.execute(my_plan);
      // move_group.move();

      // loop_rate2.sleep();

      move_group.setJointValueTarget(joint_names, joint_values_4);
      success = (move_group.plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      move_group.execute(my_plan);
      // move_group.move();

      // loop_rate2.sleep();

      // msg.data = 1;
      // pub_->publish(msg);
    }
    loop_rate2.sleep();
  }

  rclcpp::shutdown();
  return 0;
}