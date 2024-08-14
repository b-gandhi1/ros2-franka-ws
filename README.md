# ros2-franka-ws
This workspace is for using franks emika robot arm via ros2. 
Ros2 has some better integration for franka than ros1. 
A bridge between ros2 and ros1 is also being used to bring the project together. 

## 1. Source ros2 and workspace
```
cd ~/ros2-franka-ws
source /opt/ros/foxy/setup.bash
source install/setup.bash

# RUN SIMULATION
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=dont_care use_fake_hardware:=true
```
## 2. Gripper commands
```
# terminal 1: 
ros2 launch franka_gripper gripper.launch.py robot_ip:=173.16.0.2

# terminal 2: 

# open gripper
ros2 action send_goal /panda_gripper/gripper_action control_msgs/action/GripperCommand "{command: {position: 0.022, max_effort: -1}}"

# secure gripper
ros2 action send_goal /panda_gripper/grasp franka_msgs/action/Grasp "{width: 0.0, force: 40.0, speed: 0.1,  epsilon:{inner: 0.08, outer: 0.08}}"
```
## 3. Commands to get joint angles (for setting up new paths)
ONLY NEEDS SETTING ONCE
```
# terminal 1: 
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=173.16.0.2 

# terminal 2: 
ros2 topic echo /joint_states
```
Get first 7 positions and update the cpp file.

## 4. Running the publisher
``` 
ros2 run motionmannequin external_force # needs running once to set joint torques
# above line enables issues around payload for moving heavy mannequin head!

# and finally run the motion file:
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2

# OR - use shell script for above: 
sh run_prog_commands.sh
```

# Bring Franka to Home Position: 
```
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=d173.16.0.2
```
Then in rviz select `motion planning` > under goal pose select `ready` > `plan and execute` > close rviz and terminal > head back to franka desk > lock joints > shutdown 