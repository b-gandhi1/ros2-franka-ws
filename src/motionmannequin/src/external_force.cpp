// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <cmath>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

// #include "haptic_connect/examples_common.h"

/**
 * @example generate_joint_position_motion.cpp
 * An example showing how to generate a joint position motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    franka::Robot robot("173.16.0.2",franka::RealtimeConfig::kEnforce);
    //setDefaultBehavior(robot);

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{30.0, 30.0, 28.0, 28.0, 26.0, 24.0, 22.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{30.0, 30.0, 28.0, 28.0, 26.0, 24.0, 22.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{60.0, 60.0, 60.0, 65.0, 65.0, 65.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{60.0, 60.0, 60.0, 65.0, 65.0, 65.0}});
        // {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        // {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        // {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        // {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  rclcpp::shutdown();
  return 0;
}
