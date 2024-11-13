ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.0, force: 40.0, speed: 0.1,  epsilon:{inner: 0.08, outer: 0.08}}" # SECURE GRIPPER
