#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group_demo");

class FrankaSubscriber
{
public:
    FrankaSubscriber(rclcpp::Node::SharedPtr &node_)
    {
        sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/franka/joint_states", 10, std::bind(&FrankaSubscriber::topic_callback, this, std::placeholders::_1));
    }
    // Eigen::VectorXd joint_positions;
    std::vector<double> joint_positions = {0,0,0,0,0,0,0};
    int fhfh;

private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) 
    {
        joint_positions[0] = msg->position[0];
        joint_positions[1] = msg->position[1];
        joint_positions[2] = msg->position[2];
        joint_positions[3] = msg->position[3];
        joint_positions[4] = msg->position[4];
        joint_positions[5] = msg->position[5];
        joint_positions[6] = msg->position[6];

    }    
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ = std::make_shared<rclcpp::Node>("endeffpose");
    rclcpp::WallRate loop_rate(100);

    // Class object
    FrankaSubscriber franka_obj(node_);

    // Robot model loader
    robot_model_loader::RobotModelLoader robot_model_loader(node_);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform("panda_link8");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node_);
    std::thread([&executor]()
                { executor.spin(); })
        .detach();

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr endeff_publisher_ =
        node_->create_publisher<geometry_msgs::msg::PoseStamped>("franka_ee_pos", 10);
    auto pub_2 = node_->create_publisher<std_msgs::msg::Float64MultiArray>("franka_trl_rot", 10);

    const std::string PLANNING_GROUP_LEFT = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_cont(node_, PLANNING_GROUP_LEFT);

    std_msgs::msg::Float64MultiArray franka_msg;
    geometry_msgs::msg::PoseStamped current_pose_cont;

    while (rclcpp::ok())
    {
        // Publishing the current full pose of the end-effector
        current_pose_cont = move_group_cont.getCurrentPose();
        endeff_publisher_->publish(current_pose_cont);

        // Publishing the rotation matrix
        robot_state->setJointGroupPositions("panda_arm", franka_obj.joint_positions);
        robot_state->updateLinkTransforms();
        RCLCPP_INFO_STREAM(LOGGER, "Translation: \n" << end_effector_state.translation() << "\n");
        RCLCPP_INFO_STREAM(LOGGER, "Rotation: \n" << end_effector_state.rotation() << "\n");

        // and then publish
        franka_msg.data.clear();
        franka_msg.data.push_back(end_effector_state.translation().x());
        franka_msg.data.push_back(end_effector_state.translation().y());
        franka_msg.data.push_back(end_effector_state.translation().z());
        franka_msg.data.push_back(end_effector_state.rotation()(0,0));
        franka_msg.data.push_back(end_effector_state.rotation()(0,1));
        franka_msg.data.push_back(end_effector_state.rotation()(0,2));
        franka_msg.data.push_back(end_effector_state.rotation()(1,0));
        franka_msg.data.push_back(end_effector_state.rotation()(1,1));
        franka_msg.data.push_back(end_effector_state.rotation()(1,2));
        franka_msg.data.push_back(end_effector_state.rotation()(2,0));
        franka_msg.data.push_back(end_effector_state.rotation()(2,1));
        franka_msg.data.push_back(end_effector_state.rotation()(2,2));
        pub_2->publish(franka_msg);

        loop_rate.sleep();
    }
    return 0;
}