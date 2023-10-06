#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("endeffpose");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr endeff_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("franka_ee_pos", 10);

    const std::string PLANNING_GROUP_LEFT = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_cont(node_, PLANNING_GROUP_LEFT);

    while (rclcpp::ok())
    {
        geometry_msgs::msg::PoseStamped current_pose_cont;
        current_pose_cont = move_group_cont.getCurrentPose();
        endeff_publisher_->publish(current_pose_cont);
    }
    return 0;
}