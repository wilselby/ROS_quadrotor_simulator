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


#include "quad_joystick_interface/joy.h"


Joy::Joy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  //Publishing message type CommandRollPitchYawrateThrust on topic "command/roll_pitch_yawrate_thrust"
  ctrl_pub_ = nh_.advertise<mav_msgs::CommandRollPitchYawrateThrust> (
    "command/roll_pitch_yawrate_thrust", 10);
  trajectory_pub = nh.advertise<mav_msgs::CommandTrajectory>("command/trajectory", 10);

  //Initialize command/roll_pitch_yawrate_thrust message
  control_msg_.roll = 0;	// rad
  control_msg_.pitch = 0;	// rad
  control_msg_.yaw_rate = 0;	// rad/s
  control_msg_.thrust = 0;	// N
  current_yaw_vel_ = 0;		// rad/s

  trajectory_msg.position.x = 0.0;	// m
  trajectory_msg.position.y = 0.0;	// m 
  trajectory_msg.position.z = 0.0;	// m
  trajectory_msg.yaw = 0.0;	// rad

  // Hack for sending button statuses
  trajectory_msg.snap.x = 0.0;	// takeoff
  trajectory_msg.snap.y = 0.0;	// land
  trajectory_msg.jerk.x = 0.0;	// enable GPS
  trajectory_msg.jerk.y = 0.0;	// enable mission

  mission_mode = 0;

  //Initialize Parameters

  // Map similar to RC set-up
  pnh.param("axis_roll_", axes_.roll, 3);	// RS <->
  pnh.param("axis_pitch_", axes_.pitch, 4);	// RS up/down
  pnh.param("axis_thrust_", axes_.thrust, 1);	// LS up/down
  pnh.param("axis_yaw_", axes_.yaw, 0);		// LS <->

  pnh.param("axis_direction_roll", axes_.roll_direction, -1);
  pnh.param("axis_direction_pitch", axes_.pitch_direction, 1);
  pnh.param("axis_direction_thrust", axes_.thrust_direction, 1);
  pnh.param("axis_direction_yaw", axes_.yaw_direction, -1);

  pnh.param("max_v_xy", max_.v_xy, 1.0);  // [m/s]
  pnh.param("max_roll", max_.roll, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_pitch", max_.pitch, 25.0 * M_PI / 180.0);  // [rad]
  pnh.param("max_yaw_rate", max_.rate_yaw, 50 * M_PI / 180.0);  // [rad/s]
  pnh.param("max_thrust", max_.thrust, 10.0);  // [N] 
  pnh.param("thrust_offset", max_.thrust_offset, 2.75);  // [N] 

  pnh.param("v_yaw_step", v_yaw_step_, 0.1);  // [rad/s]

  pnh.param("button_ctrl_enable_", buttons_.ctrl_enable_gps, 1); // B
  pnh.param("button_ctrl_mode_", buttons_.ctrl_enable_mission, 3); // Y
  pnh.param("button_takeoff_", buttons_.takeoff, 0);	// A
  pnh.param("button_land_", buttons_.land, 2);		// X
  pnh.param("button_ctrl_auto", buttons_.ctrl_enable_autonomous, 5);	// RB
  pnh.param("button_ctrl_3dnav", buttons_.ctrl_enable_3dnav, 4);	// LB

  namespace_ = nh_.getNamespace();

  //Subscribe to message "joy" and callback JoyCallback
  joy_sub_ = nh_.subscribe("joy", 10, &Joy::JoyCallback, this);
}

//Function to zero out control command
void Joy::StopMav() {

  control_msg_.roll = 0;
  control_msg_.pitch = 0;
  control_msg_.yaw_rate = 0;
  control_msg_.thrust = 0;

}

void Joy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  current_joy_ = *msg;

  control_msg_.roll = msg->axes[axes_.roll] * max_.roll * axes_.roll_direction;
  control_msg_.pitch = msg->axes[axes_.pitch] * max_.pitch * axes_.pitch_direction;
  control_msg_.thrust = max_.thrust_offset + (msg->axes[axes_.thrust] + 1) * max_.thrust / 2.0 * axes_.thrust_direction;
  control_msg_.yaw_rate = msg->axes[axes_.yaw] * max_.rate_yaw * axes_.yaw_direction;

  trajectory_msg.position.x = 7.5*control_msg_.pitch;
  trajectory_msg.position.y = -7.5*control_msg_.roll;	// ROS y axis pos to the left
  trajectory_msg.position.z = 0.1*(msg->axes[axes_.thrust] * max_.thrust / 2.0 * axes_.thrust_direction);	
  trajectory_msg.yaw = -control_msg_.yaw_rate;	

  // Hack for sending button states
  trajectory_msg.snap.x = msg->buttons[buttons_.takeoff];	// takeoff
  trajectory_msg.snap.y = msg->buttons[buttons_.land];	// land
  gps_mode = msg->buttons[buttons_.ctrl_enable_gps];	// enable GPS
  trajectory_msg.jerk.x = msg->buttons[buttons_.ctrl_enable_gps];
  mission_mode = msg->buttons[buttons_.ctrl_enable_mission];	// enable mission
  trajectory_msg.jerk.y = msg->buttons[buttons_.ctrl_enable_mission];
  auto_mode = msg->buttons[buttons_.ctrl_enable_autonomous];	// enable auto
  trajectory_msg.jerk.z = msg->buttons[buttons_.ctrl_enable_autonomous];
  trajectory_msg.snap.z = msg->buttons[buttons_.ctrl_enable_3dnav];	// 3D nav

  //Control message header information
  ros::Time update_time = ros::Time::now();
  control_msg_.header.stamp = update_time;
  control_msg_.header.frame_id = "quad_joystick_attitude_frame";

  trajectory_msg.header.stamp = update_time;
  trajectory_msg.header.frame_id = "quad_joystick_position_frame";

  Publish();	//Publish control command

}

//Function to publish control command
void Joy::Publish() {

  gps_mode_switch.UpdateSwitchValue(gps_mode);
  mission_mode_switch.UpdateSwitchValue(mission_mode);
  auto_mode_switch.UpdateSwitchValue(auto_mode);

  //Enable GPS mode
  if(gps_mode_switch.GetSwitchValue()){
    trajectory_msg.jerk.x = 1;	//Flag for position_controller_node
  }
  else {
    trajectory_msg.jerk.x = 0;  //Flag for position_controller_node
    ctrl_pub_.publish(control_msg_);
  }  

  trajectory_pub.publish(trajectory_msg); 

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "quad_joy_interface");	//Specify node name
  Joy joy;

  ros::spin();

  return 0;
}
