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

#include "attitude_controller_node.h"

#include "quad_control/parameters_ros.h"


// Variables
mav_msgs::CommandRollPitchYawrateThrust current_control_cmd_;
sensor_msgs::Imu current_imu_;

rotors_control::VehicleParameters vehicle_parameters_;


namespace quad_control {

AttitudeControllerNode::AttitudeControllerNode(){

  InitializeParams();

  ros::NodeHandle nh;

  // Subscribers
  cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe("command/roll_pitch_yawrate_thrust", 10, &AttitudeControllerNode::AttitudeCommandCallback, this);

  imu_sub_ = nh.subscribe("imu", 10, &AttitudeControllerNode::IMUCallback, this);

  //Publisher
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::CommandMotorSpeed>("command/motor_speed", 10);
}


AttitudeControllerNode::~AttitudeControllerNode() {}

void AttitudeControllerNode::InitializeParams(){

  ros::NodeHandle pnh("~");   

  GetVehicleParameters(pnh, &vehicle_parameters_);

  attitude_controller_.InitializeParameters(pnh);

  ROS_INFO_ONCE("Attitude_controller_node Paramters Initialized.");

}

// Callbacks
void AttitudeControllerNode::AttitudeCommandCallback(
    const mav_msgs::CommandRollPitchYawrateThrustConstPtr& attitude_reference_msg) {

  ROS_INFO_ONCE("Attitude_controller_node got first Joystick message.");

  current_control_cmd_ = *attitude_reference_msg;

  ROS_DEBUG("I heard: [%0.3f]", attitude_reference_msg->thrust);
 
}

void AttitudeControllerNode::Publish() {

  mav_msgs::CommandMotorSpeedPtr cmd_motors_msg(new mav_msgs::CommandMotorSpeed);

  cmd_motors_msg->motor_speed.clear();

  for (int i = 0; i < ref_motor_velocities.size(); i++) {
    cmd_motors_msg->motor_speed.push_back(ref_motor_velocities[i]);
  }

  ros::Time update_time = ros::Time::now();
  cmd_motors_msg->header.stamp = update_time;
  cmd_motors_msg->header.frame_id = "quad_altitude_ctrl_frame";

  motor_velocity_reference_pub_.publish(cmd_motors_msg);

}

void AttitudeControllerNode::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

  ROS_INFO_ONCE("Attitude_controller_node got first IMU message.");

  current_imu_ =  *imu_msg;

  ROS_DEBUG("I heard: [%0.3f]", imu_msg->orientation.w);

  ref_motor_velocities.resize(4);
  des_control_output.resize(4);
  des_rate_output.resize(4);

  //Atttidue Control Loop
  attitude_controller_.CalculateAttitudeControl(current_control_cmd_, current_imu_, &des_rate_output);

  //Angular Rate Control Loop
  attitude_controller_.CalculateRateControl(current_control_cmd_, current_imu_, des_rate_output, &des_control_output);

  //Control Input to Motor Input Mapping
  attitude_controller_.CalculateMotorCommands(des_control_output, &ref_motor_velocities);

  Publish();
 
}

}

//Main
int main(int argc, char** argv) {

  ros::init(argc, argv, "attitude_controller_node");

  quad_control::AttitudeControllerNode attitude_controller_node;

  ros::spin();

  return 0;
}
