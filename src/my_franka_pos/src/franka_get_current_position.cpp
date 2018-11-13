// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <unistd.h>



/**
 * @example generate_cartesian_velocity_motion.cpp
 * An example showing how to generate a Cartesian velocity motion.
 *
 * @warning Before executing this example, make sure there is enough space in front of the robot.
 */

int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_current_position_publisher");
  ros::NodeHandle node_handle("~");

  std::string robot_ip = "130.230.37.115";
//  node_handle.getParam("robot_ip", robot_ip);

  double publish_rate = 30.0;
//  node_handle.getParam("publish_rate", publish_rate);

  ros::Rate rate(publish_rate);
  std_msgs::Float64MultiArray states;
//  sensor_msgs::JointState states;
//  states.effort.resize(joint_names.size());
//  states.name.resize(joint_names.size());
//  states.position.resize(joint_names.size());
//  states.velocity.resize(joint_names.size());

  ros::Publisher publisher = node_handle.advertise<std_msgs::Float64MultiArray>("franka_current_position", 1);

  try {
    franka::Robot robot(robot_ip);

    robot.read([&](const franka::RobotState& robot_state) {
      for (size_t i = 12; i < 15; i++) {
        states.data.push_back(robot_state.O_T_EE_d[i]);
      }
      publisher.publish(states);
      ros::spinOnce();
      rate.sleep();
      return ros::ok();
    });

  } catch (const franka::Exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return -1;
  }

  return 0;
}
