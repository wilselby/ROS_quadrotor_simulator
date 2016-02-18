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

#define wrap_180(x) (x < -M_PI ? x+(2*M_PI) : (x > M_PI ? x - (2*M_PI): x))

#include "quad_control/quad_controller.h"

#include "quad_control/parameters_ros.h"

namespace quad_control {

//Utility functions
ControllerUtility::ControllerUtility(void): switchValue(false), prevInput(false) { }
ControllerUtility::~ControllerUtility() {}

double ControllerUtility::map(double x, double in_min, double in_max, double out_min, double out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double ControllerUtility::limit( double in, double min, double max){
    if(in < min){
      in = min;
    }
    if( in > max){
      in = max;
    }
    return in;
}

bool ControllerUtility::GetSwitchValue(void){

        return switchValue;
}

bool ControllerUtility::UpdateSwitchValue(bool currInput)
{
        if (currInput != prevInput)
        {
            if (currInput)
            {
                switchValue = !switchValue;
            }

            prevInput = currInput;
        }

        return switchValue;
}


Eigen::Vector3d ControllerUtility::rotateGFtoBF(double GF_x, double GF_y, double GF_z, double GF_roll, double GF_pitch, double GF_yaw){ 

  Eigen::Matrix3d R_roll;
  Eigen::Matrix3d R_pitch;
  Eigen::Matrix3d R_yaw;
  Eigen::Matrix3d Rot;

  Eigen::Vector3d GF_(GF_x, GF_y, GF_z);
  Eigen::Vector3d BF_;

  R_roll << 1, 0, 0, 0, cos(GF_roll), -sin(GF_roll), 0, sin(GF_roll), cos(GF_roll);
  R_pitch << cos(GF_pitch), 0 , sin(GF_pitch), 0, 1, 0, -sin(GF_pitch), 0, cos(GF_pitch);
  R_yaw << cos(GF_yaw), -sin(GF_yaw), 0, sin(GF_yaw), cos(GF_yaw), 0, 0, 0, 1;

  Rot = R_yaw * R_pitch * R_roll;

  BF_ = GF_.transpose()*Rot;

  return BF_.transpose();

} 

//Position Controller
PositionController::PositionController() {}
PositionController::~PositionController() {}


void PositionController::InitializeParameters(const ros::NodeHandle& pnh){

  //General parameters
  last_time = ros::Time::now();

  //Altitude PID
  rotors_control::GetRosParameter(pnh,"x_PID/P", x_KP, &x_KP);
  rotors_control::GetRosParameter(pnh,"x_PID/I", x_KI, &x_KI);
  rotors_control::GetRosParameter(pnh,"x_PID/I_max", x_KI_max, &x_KI_max);
  rotors_control::GetRosParameter(pnh,"x_PID/D", x_KD, &x_KD);

  rotors_control::GetRosParameter(pnh,"y_PID/P", y_KP, &y_KP);
  rotors_control::GetRosParameter(pnh,"y_PID/I", y_KI, &y_KI);
  rotors_control::GetRosParameter(pnh,"y_PID/I_max", y_KI_max, &y_KI_max);
  rotors_control::GetRosParameter(pnh,"y_PID/D", y_KD, &y_KD);

  rotors_control::GetRosParameter(pnh,"z_PID/P", z_KP, &z_KP);
  rotors_control::GetRosParameter(pnh,"z_PID/I", z_KI, &z_KI);
  rotors_control::GetRosParameter(pnh,"z_PID/I_max", z_KI_max, &z_KI_max);
  rotors_control::GetRosParameter(pnh,"z_PID/D", z_KD, &z_KD);

  rotors_control::GetRosParameter(pnh,"yaw_PID/P", yaw_KP, &yaw_KP);
  rotors_control::GetRosParameter(pnh,"yaw_PID/I", yaw_KI, &yaw_KI);
  rotors_control::GetRosParameter(pnh,"yaw_PID/I_max", yaw_KI_max, &yaw_KI_max);
  rotors_control::GetRosParameter(pnh,"yaw_PID/D", yaw_KD, &yaw_KD);

  rotors_control::GetRosParameter(pnh,"mass", mass, &mass);

  x_er = 0;
  y_er = 0;
  z_er = 0;
  yaw_er = 0;
  x_er_sum = 0;
  y_er_sum = 0;
  z_er_sum = 0;
  yaw_er_sum = 0;
  roll_des = 0;
  pitch_des = 0;
  thrust_des = 0;
  yaw_des = 0;

}

void PositionController::CalculatePositionControl(mav_msgs::CommandTrajectory wp, nav_msgs::Odometry current_gps, mav_msgs::CommandRollPitchYawrateThrust *des_attitude_output){


  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_gps.pose.pose.orientation, q);
  tf::Matrix3x3(q).getRPY(gps_roll, gps_pitch, gps_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", gps_roll, gps_pitch, gps_yaw);

  // Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;

  gps_x = current_gps.pose.pose.position.x;
  gps_y = current_gps.pose.pose.position.y;
  gps_z = current_gps.pose.pose.position.z;

  //X PID
  x_er = wp.position.x - gps_x;
  if(abs(x_er) < x_KI_max){
	x_er_sum = x_er_sum + x_er;
  }  
  cp = x_er * x_KP;
  ci = x_KI * dt * x_er_sum;
  cd = x_KD * current_gps.twist.twist.linear.x;
  pitch_des = (cp - cd );
  pitch_des = controller_utility_.limit(pitch_des, -15.0 * M_PI / 180.0, 15.0 * M_PI / 180.0);


  //Y PID
  y_er = wp.position.y - gps_y;
  if(abs(y_er) < y_KI_max){
	y_er_sum = y_er_sum + y_er;
  }
  cp = y_er * y_KP;
  ci = y_KI * dt * y_er_sum;
  cd = y_KD * current_gps.twist.twist.linear.y;
  roll_des = -(cp - cd);	//Positive Y axis and roll angles inversely related
  roll_des = controller_utility_.limit(roll_des, -15.0 * M_PI / 180.0, 15.0 * M_PI / 180.0);

  //Z PID
  z_er = wp.position.z - gps_z;
  if(abs(z_er) < z_KI_max){
	z_er_sum = z_er_sum + z_er;
  }  
  cp = z_er * z_KP;
  ci = z_KI * dt * z_er_sum;
  cd = z_KD * current_gps.twist.twist.linear.z;
  thrust_des = (cp - cd) + 7.84;	//Hover is ~ 7.84N (should be (mass*9.8) but that is too large
  thrust_des = controller_utility_.limit(thrust_des, 2.75, 20);

  //Yaw PID
  yaw_er = wrap_180(wp.yaw - gps_yaw);

  if(abs(yaw_er) < yaw_KI_max){
	yaw_er_sum = yaw_er_sum + yaw_er;
  }

  cp = yaw_er * yaw_KP;
  ci = yaw_KI * dt * yaw_er_sum;
  cd = yaw_KD * current_gps.twist.twist.angular.z;
  yaw_des = -(cp - cd); 
  yaw_des = controller_utility_.limit(yaw_des, -50.0 * M_PI / 180.0, 50.0 * M_PI / 180.0);

  des_attitude_cmds.roll = roll_des;	
  des_attitude_cmds.pitch = pitch_des;
  des_attitude_cmds.yaw_rate = yaw_des;
  des_attitude_cmds.thrust = thrust_des;

  *des_attitude_output = des_attitude_cmds;

}



//Attitude Controller
AttitudeController::AttitudeController() {}
/*
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
*/


AttitudeController::~AttitudeController() {}

void AttitudeController::InitializeParameters(const ros::NodeHandle& pnh){

  //General parameters
  last_time = ros::Time::now();

  //Roll PID
  rotors_control::GetRosParameter(pnh,"roll_PID/P", roll_KP, &roll_KP);
  rotors_control::GetRosParameter(pnh,"roll_PID/I", roll_KI, &roll_KI);
  rotors_control::GetRosParameter(pnh,"roll_PID/I_max", roll_KI_max, &roll_KI_max);
  rotors_control::GetRosParameter(pnh,"roll_PID/D", roll_KD, &roll_KD);

  //Pitch PID
  rotors_control::GetRosParameter(pnh,"pitch_PID/P", pitch_KP, &pitch_KP);
  rotors_control::GetRosParameter(pnh,"pitch_PID/I", pitch_KI, &pitch_KI);
  rotors_control::GetRosParameter(pnh,"pitch_PID/I_max", pitch_KI_max, &pitch_KI_max);
  rotors_control::GetRosParameter(pnh,"pitch_PID/D", pitch_KD, &pitch_KD);

  //Yaw PID
  rotors_control::GetRosParameter(pnh,"yaw_PID/P", yaw_KP, &yaw_KP);
  rotors_control::GetRosParameter(pnh,"yaw_PID/I", yaw_KI, &yaw_KI);
  rotors_control::GetRosParameter(pnh,"yaw_PID/I_max", yaw_KI_max, &yaw_KI_max);
  rotors_control::GetRosParameter(pnh,"yaw_PID/D", yaw_KD, &yaw_KD);

  roll_er = 0;
  pitch_er = 0;
  yaw_er = 0;
  roll_er_sum = 0;
  pitch_er_sum = 0;
  yaw_er_sum = 0;
  p_des = 0;
  q_des = 0;
  r_des = 0;
  yaw_target = 0;

  //Roll rate PID
  rotors_control::GetRosParameter(pnh,"p_PID/P", p_KP, &p_KP);
  rotors_control::GetRosParameter(pnh,"p_PID/I", p_KI, &p_KI);
  rotors_control::GetRosParameter(pnh,"p_PID/I_max", p_KI_max, &p_KI_max);
  rotors_control::GetRosParameter(pnh,"p_PID/D", p_KD, &p_KD);

  //Pitch rate PID
  rotors_control::GetRosParameter(pnh,"q_PID/P", q_KP, &q_KP);
  rotors_control::GetRosParameter(pnh,"q_PID/I", q_KI, &q_KI);
  rotors_control::GetRosParameter(pnh,"q_PID/I_max", q_KI_max, &q_KI_max);
  rotors_control::GetRosParameter(pnh,"q_PID/D", q_KD, &q_KD);

  //Yaw rate PID
  rotors_control::GetRosParameter(pnh,"r_PID/P", r_KP, &r_KP);
  rotors_control::GetRosParameter(pnh,"r_PID/I", r_KI, &r_KI);
  rotors_control::GetRosParameter(pnh,"r_PID/I_max", r_KI_max, &r_KI_max);
  rotors_control::GetRosParameter(pnh,"r_PID/D", r_KD, &r_KD);

  p_er = 0;
  q_er = 0;
  r_er = 0;
  p_er_sum = 0;
  q_er_sum = 0;
  r_er_sum = 0;
  x_ang_acc = 0;
  last_ang_vel_x = 0;
  y_ang_acc = 0;
  last_ang_vel_y = 0;
  z_ang_acc = 0;
  last_ang_vel_z = 0;
  U1 = 0;
  U2 = 0;
  U3 = 0;
  U4 = 0;


  //TODO Param server?
  //Control input to motor mapping  
  KT = 1.33e-05;
  Kd = 1.39e-06;
  l = .2;
  motor_lim = 890*890;

  w1 = 0;
  w2 = 0;
  w3 = 0;
  w4 = 0;
}


/*
TODO: 
Limit Integrators (ci)
Limit Control Commands (x_des)
*/
void AttitudeController::CalculateAttitudeControl(mav_msgs::CommandRollPitchYawrateThrust control_cmd_input, sensor_msgs::Imu current_imu, Eigen::VectorXd* des_rate_output){

  //Determine vector size based on number of motors
  des_rate_output->resize(4);
  
  desired_angular_rates.resize(4);

  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_imu.orientation, q);
  tf::Matrix3x3(q).getRPY(meas_roll, meas_pitch, meas_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", meas_roll, meas_pitch, meas_yaw);

  // Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;

  //Roll PID
  roll_er = control_cmd_input.roll - meas_roll;
  if(abs(roll_er) < roll_KI_max){
	roll_er_sum = roll_er_sum + roll_er;
  }  
  cp = roll_er * roll_KP;
  ci = roll_KI * dt * roll_er_sum;
  cd = roll_KD * current_imu.angular_velocity.x;
  p_des = cp - cd;

  //Pitch PID
  pitch_er = control_cmd_input.pitch - meas_pitch;
  if(abs(pitch_er) < pitch_KI_max){
	pitch_er_sum = pitch_er_sum + pitch_er;
  }
  cp = pitch_er * pitch_KP;
  ci = pitch_KI * dt * pitch_er_sum;
  cd = pitch_KD * current_imu.angular_velocity.y;
  q_des = cp - cd;

  //Yaw PID
  yaw_er = yaw_target - meas_yaw;

  if(abs(yaw_er) < yaw_KI_max){
	yaw_er_sum = yaw_er_sum + yaw_er;
  }

  cp = yaw_er * yaw_KP;
  ci = yaw_KI * dt * yaw_er_sum;
  cd = yaw_KD * current_imu.angular_velocity.z;
  r_des = cp - cd; //+/-?

  desired_angular_rates[0] = control_cmd_input.thrust;	//~450 = hovering for the hummingbord model
  desired_angular_rates[1] = p_des;
  desired_angular_rates[2] = q_des;
  desired_angular_rates[3] = r_des;

  *des_rate_output = desired_angular_rates;

}


/*
TODO: 
Limit Integrators (ci)
Limit Control Commands (U#)
*/
void AttitudeController::CalculateRateControl(mav_msgs::CommandRollPitchYawrateThrust control_cmd_input, sensor_msgs::Imu current_imu, Eigen::VectorXd des_rate_input, Eigen::VectorXd* des_control_output){

  //Determine vector size based on number of motors
  des_control_output->resize(4);
  
  desired_control_cmds.resize(4);

  //Convert quaternion to Euler angles
  tf:quaternionMsgToTF(current_imu.orientation, q);
  tf::Matrix3x3(q).getRPY(meas_roll, meas_pitch, meas_yaw);
  ROS_DEBUG("RPY = (%lf, %lf, %lf)", meas_roll, meas_pitch, meas_yaw);

  //Get simulator time
  sim_time = ros::Time::now();
  dt = (sim_time - last_time).toSec();
  if (dt == 0.0) return;

  //P PID
  p_er = des_rate_input[1] - current_imu.angular_velocity.x;
  if(abs(p_er) < p_KI_max){
	p_er_sum = p_er_sum + p_er;
  }  
  cp = p_er * p_KP;
  ci = p_KI * dt * p_er_sum;
  x_ang_acc = (current_imu.angular_velocity.x - last_ang_vel_x)/dt;
  cd = p_KD * x_ang_acc;
  U2 = cp - cd;
  last_ang_vel_x = current_imu.angular_velocity.x;

  //Q PID
  q_er = des_rate_input[2] - current_imu.angular_velocity.y;
  if(abs(q_er) < q_KI_max){
	q_er_sum = q_er_sum + q_er;
  }
  cp = q_er * q_KP;
  ci = q_KI * dt * q_er_sum;
  y_ang_acc = (current_imu.angular_velocity.y - last_ang_vel_y)/dt;
  cd = q_KD * y_ang_acc;
  U3 = cp - cd;
  last_ang_vel_y = current_imu.angular_velocity.y;


  //Yaw PID
  yaw_vel_target = des_rate_input[3]; // if using position control

  //Reset desired heading velocity based on position/RC control
  if((control_cmd_input.yaw_rate > .04) || (control_cmd_input.yaw_rate < -.04) ){
    yaw_vel_target = control_cmd_input.yaw_rate;
    yaw_target = meas_yaw;
  }
  else{
    yaw_vel_target = 0;
  }	

  r_er = yaw_vel_target - (-current_imu.angular_velocity.z);
  if(abs(r_er) < r_KI_max){
	r_er_sum = r_er_sum + r_er;
  }
  cp = r_er * r_KI;
  ci = r_KI * dt * r_er_sum;
  z_ang_acc = (-current_imu.angular_velocity.z - last_ang_vel_z)/dt;
  cd = r_KD * z_ang_acc;
  U4 = cp - cd;
  last_ang_vel_z = current_imu.angular_velocity.z;

  desired_control_cmds[0] = des_rate_input[0];
  desired_control_cmds[1] = U2;
  desired_control_cmds[2] = U3;
  desired_control_cmds[3] = U4;

  *des_control_output = desired_control_cmds;

}
void AttitudeController::CalculateMotorCommands(Eigen::VectorXd control_inputs, Eigen::VectorXd* des_rotor_velocities_output){

  //Determine vector size based on number of motors

  //desired_motor_velocities.resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  desired_motor_velocities.resize(4);

  U1 = control_inputs[0];
  U2 = control_inputs[1];
  U3 = control_inputs[2];
  U4 = control_inputs[3];

  //Control input to motor mapping  
  w1 = U1/(4*KT) - U3/(2*KT*l) + U4/(4*Kd);
  w2 = U1/(4*KT) - U2/(2*KT*l) - U4/(4*Kd);
  w3 = U1/(4*KT) + U3/(2*KT*l) + U4/(4*Kd);
  w4 = U1/(4*KT) + U2/(2*KT*l) - U4/(4*Kd);

  // Limit Based on Motor Parameters
  w1 = controller_utility_.limit(w1, 0, motor_lim);
  w2 = controller_utility_.limit(w2, 0, motor_lim);
  w3 = controller_utility_.limit(w3, 0, motor_lim);
  w4 = controller_utility_.limit(w4, 0, motor_lim);

  //Calculate motor speeds
  w1 = sqrt(w1);
  w2 = sqrt(w2);
  w3 = sqrt(w3);
  w4 = sqrt(w4);  

  desired_motor_velocities[0] = w1;
  desired_motor_velocities[1] = w2;
  desired_motor_velocities[2] = w3;
  desired_motor_velocities[3] = w4;

  *des_rotor_velocities_output = desired_motor_velocities;

}


}


/*
  current_control_cmd_ desired angles/thrust
  control_msg_.roll = 0;	// rad
  control_msg_.pitch = 0;	// rad
  control_msg_.yaw_rate = 0;	// rad/s
  control_msg_.thrust = 0;	// N

  current_imu_ actual angles
  imu_message_.orientation.w = 1;
  imu_message_.orientation.x = 0;
  imu_message_.orientation.y = 0;
  imu_message_.orientation.z = 0;
  imu_message_.linear_acceleration.x = linear_acceleration_I[0]; m/s^2
  imu_message_.linear_acceleration.y = linear_acceleration_I[1];
  imu_message_.linear_acceleration.z = linear_acceleration_I[2];
  imu_message_.angular_velocity.x = angular_velocity_I[0];	rad/s
  imu_message_.angular_velocity.y = angular_velocity_I[1];
  imu_message_.angular_velocity.z = angular_velocity_I[2];
*/




