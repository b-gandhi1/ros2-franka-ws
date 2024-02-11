#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

// #include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

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
    //   std::vector<double> joint_values = { 1.923,-1.723, -1.4935, -1.945, -2.178, 1.996, 1.562 };

    std::vector<double> joint_values1,joint_values2,joint_values3,joint_values4;
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
            joint_values1 = {}; // UP

            move_group.setJointValueTarget(joint_names, joint_values1);
            move_group.setMaxVelocityScalingFactor(0.1);
            success = (move_group.plan(my_plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
            move_group.execute(my_plan);
            // move_group.move();

            // loop_rate2.sleep();

            joint_values2 = {}; // DOWN

            move_group.setJointValueTarget(joint_names, joint_values2);
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