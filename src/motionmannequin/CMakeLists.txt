cmake_minimum_required(VERSION 3.5)
project(motionmannequin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface)
find_package(moveit_servo)
find_package(moveit_msgs)
find_package(moveit)
find_package(rosidl_default_generators REQUIRED)
find_package(franka_msgs)

add_executable(motion_pillow src/motion_pillow.cpp)
add_executable(endeffpose src/endeffpose.cpp)
add_executable(external_force src/external_force.cpp)

target_link_libraries(external_force /usr/lib64/libfranka.so)

ament_target_dependencies(motion_pillow rclcpp std_msgs geometry_msgs moveit moveit_ros_planning_interface)
ament_target_dependencies(endeffpose rclcpp std_msgs geometry_msgs moveit moveit_ros_planning_interface)
ament_target_dependencies(external_force rclcpp)

install(TARGETS
  motion_pillow
  endeffpose
  external_force
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
