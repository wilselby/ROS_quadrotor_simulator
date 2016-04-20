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

#include <thread>
#include <chrono>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>
#include <fstream>
#include <iostream>

#include <std_srvs/Empty.h>

#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandTrajectory.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/conversions.h>
#include <planning_msgs/WayPoint.h>
#include <planning_msgs/eigen_planning_msgs.h>
#include <planning_msgs/conversions.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include "quad_control/quad_controller.h"

namespace quad_control {

class WaypointWithTime {
 public:
  WaypointWithTime()
      : waiting_time(0) {
  }

  WaypointWithTime(double t, float x, float y, float z, float yaw)
      : waiting_time(t) {
    wp.position.x = x;
    wp.position.y = y;
    wp.position.z = z;
    wp.yaw = yaw;
  }

  mav_msgs::CommandTrajectory wp;
  double waiting_time;
  const float DEG_2_RAD = M_PI / 180.0;

  std::vector<quad_control::WaypointWithTime> Read_waypoints(std::vector<quad_control::WaypointWithTime> waypoints);
};

class WaypointPublisherNode {
 public:
  WaypointPublisherNode();
  ~WaypointPublisherNode();

  void InitializeParams();
//  void Publish();

 private:

  //Subscribers
  ros::Subscriber cmd_pos_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber cmd_threednav_sub_;

  //Publisher
  ros::Publisher trajectory_pub;

  //Waypoint variables
  planning_msgs::EigenWayPoint current_waypoint_;
  planning_msgs::WayPoint desired_waypoint;
  mav_msgs::EigenCommandTrajectory command_trajectory;
  mav_msgs::EigenCommandTrajectory threedNav_trajectory;
  mav_msgs::CommandTrajectory desired_wp;
  nav_msgs::Odometry current_gps_;

  //General
  tf::Quaternion q;
  double gps_roll, gps_pitch, gps_yaw;

  ControllerUtility control_mode;
  ControllerUtility auto_mode;
  ControllerUtility threednav_mode;
 
  WaypointWithTime waypoint_utility;
  std::vector<WaypointWithTime> waypoints;
  Eigen::Vector3d waypointBF;
  geometry_msgs::Twist cmd_vel;

  const float DEG_2_RAD = M_PI / 180.0;
  int waypoints_read;
  int published;
  size_t i;
  double start_time;
  double current_time;

  void CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& command_trajectory_msg);
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
  void CommandVelCallback(const geometry_msgs::TwistConstPtr& command_velocity_msg);
  void threedNavCallback(const mav_msgs::CommandTrajectoryConstPtr& threed_nav_msg);

};


}
