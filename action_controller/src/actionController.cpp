#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <pthread.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <action_controller/MultiDofFollowJointTrajectoryAction.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/CommandTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class Controller{
private:
	typedef actionlib::ActionServer<action_controller::MultiDofFollowJointTrajectoryAction> ActionServer;
	typedef ActionServer::GoalHandle GoalHandle;
public:
	Controller(ros::NodeHandle &n) :
		node_(n),
		action_server_(node_, "multi_dof_joint_trajectory_action",
				boost::bind(&Controller::goalCB, this, _1),
				boost::bind(&Controller::cancelCB, this, _1),
				false),
				has_active_goal_(false)
{
		created=0;
		empty.linear.x=0;
		empty.linear.y=0;
		empty.linear.z=0;
		empty.angular.z=0;
		empty.angular.y=0;
		empty.angular.x=0;
		pub_topic = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
		trajectory_pub = node_.advertise<mav_msgs::CommandTrajectory>("/cmd_3dnav", 10);
		action_server_.start();
		printf("\n\n Node ready! \n\n");
}
private:
	ros::NodeHandle node_;
	ActionServer action_server_;
	ros::Publisher pub_topic;

	ros::Publisher trajectory_pub;
  	mav_msgs::CommandTrajectory desired_wp;
  
  	tf::Quaternion q;
  	double des_roll, des_pitch, des_yaw;

	geometry_msgs::Twist empty;
	geometry_msgs::Transform_<std::allocator<void> > lastPosition;
	ros::Duration lastTime;
	ros::Duration currentTime;
	double dt;
	geometry_msgs::Twist cmd;
	pthread_t trajectoryExecutor;
	int created;

	bool has_active_goal_;
	GoalHandle active_goal_;
	trajectory_msgs::MultiDOFJointTrajectory_<std::allocator<void> > toExecute;

  	double current_time; 
  	double start_time; 

	void cancelCB(GoalHandle gh){
		if (active_goal_ == gh)
		{
			// Stops the controller.
			if(created){
				printf("Stop thread \n");
				pthread_cancel(trajectoryExecutor);
				created=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}
	}

	void goalCB(GoalHandle gh){
		if (has_active_goal_)
		{
			// Stops the controller.
			if(created){
				pthread_cancel(trajectoryExecutor);
				created=0;
			}
			pub_topic.publish(empty);

			// Marks the current goal as canceled.
			active_goal_.setCanceled();
			has_active_goal_ = false;
		}

		gh.setAccepted();
		active_goal_ = gh;
		has_active_goal_ = true;
		toExecute = gh.getGoal()->trajectory;

		//controllore solo per il giunto virtuale Base
		if(pthread_create(&trajectoryExecutor, NULL, threadWrapper, this)==0){
			created=1;
			printf("Thread for trajectory execution created \n");
		} else {
			printf("Thread creation failed! \n");
		}

	}

	static void* threadWrapper(void* arg) {
		Controller * mySelf=(Controller*)arg;
		mySelf->executeTrajectory();
		return NULL;
	}

	void executeTrajectory(){
		if(toExecute.joint_names[0]=="virtual_joint" && toExecute.points.size()>0){
			for(int k=0; k<toExecute.points.size(); k++){

				geometry_msgs::Transform_<std::allocator<void> > point=toExecute.points[k].transforms[0];

				desired_wp.position.x = point.translation.x;
  				desired_wp.position.y = point.translation.y;
  				desired_wp.position.z = point.translation.z;

				//Convert quaternion to Euler angles
  				tf:quaternionMsgToTF(point.rotation, q);
  				tf::Matrix3x3(q).getRPY(des_roll, des_pitch, des_yaw);
  				desired_wp.yaw = des_yaw;

				desired_wp.jerk.x = 1;

				if(k==0){
					start_time = ros::Time::now().toSec();
					current_time = ros::Time::now().toSec();

					desired_wp.header.stamp = ros::Time::now();
        				desired_wp.header.frame_id = "3dnav_action_frame";
					trajectory_pub.publish(desired_wp);
				}
				else{

					while( (current_time-start_time) < toExecute.points[k].time_from_start.toSec() ){
					current_time = ros::Time::now().toSec();
					}
					desired_wp.header.stamp = ros::Time::now();
        				desired_wp.header.frame_id = "3dnav_action_frame";
					trajectory_pub.publish(desired_wp);
				}

/*
				geometry_msgs::Transform_<std::allocator<void> > punto=toExecute.points[k].transforms[0];
				currentTime = toExecute.points[k].time_from_start;
				bool eseguito=true;
				if(k!=0){
					dt = currentTime.toSec() - lastTime.toSec();
					eseguito=publishTranslationComand(punto,false,dt);
					if(k==(toExecute.points.size()-1)){
						if(!eseguito) publishTranslationComand(punto,true,dt);
						publishRotationComand(punto,false);
					}
				} else {
					publishRotationComand(punto,true);
				}
				pub_topic.publish(empty);
				//aggiorno start position
				if(eseguito){
					lastPosition.translation=punto.translation;
					lastPosition.rotation=punto.rotation;
					lastTime = toExecute.points[k].time_from_start;
				}
*/
			}
		}
		active_goal_.setSucceeded();
		has_active_goal_=false;
		created=0;

	}
	bool publishTranslationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool anyway, double dt){
		//creazione comando di traslazione
		cmd.linear.x=(punto.translation.x-lastPosition.translation.x)/dt;
		cmd.linear.y=(punto.translation.y-lastPosition.translation.y)/dt;
		cmd.linear.z=(punto.translation.z-lastPosition.translation.z)/dt;
		cmd.angular.x=cmd.angular.y=cmd.angular.z=0;

		if(anyway || cmd.linear.x>=0.05 || cmd.linear.y>=0.05 || cmd.linear.z>=0.05){
			printPositionInfo();
			printCmdInfo();
			pub_topic.publish(cmd);
			//tempo d'esecuzione
			ros::Duration(1.0).sleep();
			return true;
		}
		return false;
	}

	void publishRotationComand(geometry_msgs::Transform_<std::allocator<void> > punto, bool start){
		//comando di allineamento, permesse solo rotazioni sull'asse z
		cmd.linear.x=cmd.linear.y=cmd.linear.z=cmd.angular.x=cmd.angular.y=0;
		//start = true --> devo tornare nell'orientazione 0
		//start = false --> devo arrivare al'orientazione punto.rotation.z
		cmd.angular.z=(start?0-punto.rotation.z:punto.rotation.z);

		printCmdInfo();

		double sleep=cmd.angular.z*3.0; //tempo necessario a tornare nella giusta orientazione
		if(sleep<0) sleep=-sleep;
		pub_topic.publish(cmd);
		ros::Duration(sleep).sleep();
		cmd.angular.z=0;
	}

	void printPositionInfo(){
		ROS_INFO_STREAM("Start Position: ["<<lastPosition.translation.x<<
				", "<<lastPosition.translation.y<<
				", "<<lastPosition.translation.z<<"] "<<
				"[ "<<lastPosition.rotation.x<<
				", "<<lastPosition.rotation.y<<
				", "<<lastPosition.rotation.z<<" ] ");
		printf(" Start Position: [%.2f,%.2f,%.2f] [%.2f,%.2f,%.2f] \n",lastPosition.translation.x,lastPosition.translation.y,lastPosition.translation.z,lastPosition.rotation.x,lastPosition.rotation.y,lastPosition.rotation.z);
	}

	void printCmdInfo(){
		ROS_INFO_STREAM("cmd to execute: "<<"x:"<<cmd.linear.x
				<<" y: " << cmd.linear.y
				<<" z: " << cmd.linear.z
				<<" rX: " << cmd.angular.x
				<<" rY: " << cmd.angular.y
				<<" rZ: " << cmd.angular.z);
		printf("cmd to execute: X:%.2f y:%.2f z:%.2f rx:%.2f ry:%.2f rz:%.2f \n", cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z);
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "action_controller_node");
	ros::NodeHandle node;//("~");
	Controller control(node);

	ros::spin();

	return 0;
}
