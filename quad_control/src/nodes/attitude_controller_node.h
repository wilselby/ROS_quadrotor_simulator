/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "quad_control/quad_controller.h"

namespace quad_control {

class AttitudeControllerNode {
 public:
  AttitudeControllerNode();
  ~AttitudeControllerNode();

  void InitializeParams();
  void Publish();

 private:

  AttitudeController attitude_controller_;

  std::string namespace_;
  nav_msgs::Odometry current_gps;

  //Subscribers
  ros::Subscriber cmd_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber imu_sub_;
  ros::Publisher motor_velocity_reference_pub_;

  //Control variables
  Eigen::VectorXd des_rate_output;
  Eigen::VectorXd des_control_output;
  Eigen::VectorXd ref_motor_velocities;
  
  void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void AttitudeCommandCallback(const mav_msgs::CommandRollPitchYawrateThrustConstPtr& attitude_reference_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void CalculateMotorCommands(Eigen::VectorXd* motor_velocities);

  };
}

