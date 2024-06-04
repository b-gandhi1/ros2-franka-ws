ros2 run motionmannequin external_force # needs running once to set joint torques
# above line enables issues around payload for moving heavy mannequin head!

# and finally run the motion file:
ros2 launch motionmannequin motionmannequin.launch.py robot_ip:=173.16.0.2
