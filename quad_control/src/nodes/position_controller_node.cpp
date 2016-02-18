/*
 * Copyright 2015 Wil Selby
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

//TODO debug messages

#include "position_controller_node.h"

#include "quad_control/parameters_ros.h"


namespace quad_control {

PositionControllerNode::PositionControllerNode(){

  InitializeParams();

  ros::NodeHandle nh;

  // Subscribers
  cmd_trajectory_sub_ = nh.subscribe("command/waypoint", 10, &PositionControllerNode::WaypointCallback, this);
  odometry_sub_ = nh.subscribe("ground_truth/odometry", 10, &PositionControllerNode::OdometryCallback, this);

  //Publishing message type CommandRollPitchYawrateThrust on topic "command/roll_pitch_yawrate_thrust"
  ctrl_pub_ = nh.advertise<mav_msgs::CommandRollPitchYawrateThrust>("command/roll_pitch_yawrate_thrust", 10);

}


PositionControllerNode::~PositionControllerNode() {}

void PositionControllerNode::InitializeParams(){

  ros::NodeHandle pnh("~");   

  GetVehicleParameters(pnh, &vehicle_parameters_);

  position_controller_.InitializeParameters(pnh);
  
  wp.position.x = 0.0;
  wp.position.y = 0.0;
  wp.position.z = 0.5;
  wp.yaw = 0.0;

  ROS_INFO_ONCE("Position_controller_node Paramters Initialized.");

}

void PositionControllerNode::Publish(){

    //Control message header information
    ros::Time update_time = ros::Time::now();
    control_msg_.header.stamp = update_time;
    control_msg_.header.frame_id = "quad_position_ctrl_frame";

    ctrl_pub_.publish(control_msg_);

}


// Callbacks
void PositionControllerNode::WaypointCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg){

  ROS_INFO_ONCE("Position_controller_node got first Waypoint message.");

  // Update desired waypoint
  wp = *trajectory_reference_msg;  
 
}

void PositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg){

  ROS_INFO_ONCE("Position_controller_node got first GPS message.");

  current_gps_ = *odometry_msg;

  //Publish if in GPS mode
  if(wp.jerk.x){

    //Position Control Loop
    position_controller_.CalculatePositionControl(wp, current_gps_, &control_msg_);

    Publish();

    ROS_INFO_ONCE("GPS MODE");

  }

 
}

}

//Main
int main(int argc, char** argv) {

  ros::init(argc, argv, "position_controller_node");

  quad_control::PositionControllerNode position_controller_node;

  ros::spin();

  return 0;
}
