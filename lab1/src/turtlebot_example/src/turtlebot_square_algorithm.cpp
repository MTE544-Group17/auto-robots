//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 1
// It outlines the basic setup of a ros node and the various
// inputs and outputs.
//
// Author: James Servos
//
// //////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


//Callback function for the Position topic
//void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
//{
	//This function is called when a new position message is received

//	double X = msg->pose.pose.position.x; // Robot X position
//	double Y = msg->pose.pose.position.y; // Robot Y position
//	double Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

//}

double X = 0;
double Y = 0;
double Yaw = 0;

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	//This function is called when a new position message is received

	X = msg->pose.pose.position.x; // Robot X position
	Y = msg->pose.pose.position.y; // Robot Y position
 	Yaw = tf::getYaw(msg->pose.pose.orientation); // Robot Yaw

	ROS_INFO("Yaw :  %f", Yaw);

}

int main(int argc, char **argv)
{
	//Initialize the ROS framework
    ros::init(argc,argv,"main_control");
    ros::NodeHandle n;

    //Subscribe to the desired topics and assign callbacks
		ros::Subscriber pose_sub = n.subscribe("/amcl_pose", 1, pose_callback);
    // ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);

		//tell the action client that we want to spin a thread by default
		MoveBaseClient ac("move_base", true);

		//wait for the action server to come up
		while(!ac.waitForServer(ros::Duration(5.0))){
		 ROS_INFO("Waiting for the move_base action server to come up");
		}

		move_base_msgs::MoveBaseGoal goal;

		for(int i = 0; i < 4; i++) {
			goal.target_pose.header.frame_id = "base_link";
			goal.target_pose.header.stamp = ros::Time::now();

			goal.target_pose.pose.position.x = 1.0;
			// goal.target_pose.pose.orientation.x = 0.7071;
			// goal.target_pose.pose.orientation.y = 0.0;
			// goal.target_pose.pose.orientation.z = 0.7071;
			// goal.target_pose.pose.orientation.w = 0.0;

			goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1.5714);

			ROS_INFO("Sending goal");
			ac.sendGoal(goal);

			ac.waitForResult();

			if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			 ROS_INFO("Hooray, the base moved 1 meter forward");
			else
			 ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

		return 0;

    // //Setup topics to Publish from this node
    // //ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    // ros::Publisher velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
		//
    // //Velocity control variable
    // geometry_msgs::Twist vel;
		//
    // //Set the loop rate
    // ros::Rate loop_rate(20);    //20Hz update rate
		//
    // while (ros::ok())
    // {
		// 	while(ros::ok() && X < 1.0) {
		// 		ROS_INFO("X: %d", X);
		// 		vel.linear.x = 0.2; // set linear speed
		//     vel.angular.z = 0.0; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		// 	while(ros::ok() && Yaw < 1.571) {
		// 		ROS_INFO("Yaw :  %f", Yaw);
		// 		vel.linear.x = 0.0; // set linear speed
		//     vel.angular.z = 0.2; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		//
		// 	while(ros::ok() && Y < 1.0) {
		// 		vel.linear.x = 0.2; // set linear speed
		//     vel.angular.z = 0.0; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		// 	while(ros::ok() && Yaw < 3.14 && Yaw > 0) {
		// 		vel.linear.x = 0.0; // set linear speed
		//     vel.angular.z = 0.2; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		//
		// 	while(ros::ok() && X > 0) {
		// 		vel.linear.x = 0.2; // set linear speed
		//     vel.angular.z = 0.0; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		// 	while(ros::ok() && (Yaw < -1.571 || Yaw > 0)) {
		// 		vel.linear.x = 0.0; // set linear speed
		//     vel.angular.z = 0.2; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		//
		// 	while(ros::ok() && Y > 0) {
		// 		vel.linear.x = 0.2; // set linear speed
		//     vel.angular.z = 0.0; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		// 	while(ros::ok() && Yaw < 0) {
		// 		vel.linear.x = 0.0; // set linear speed
		//     vel.angular.z = 0.2; // set angular speed
		// 		loop_rate.sleep(); //Maintain the loop rate
	  //   	ros::spinOnce();   //Check for new messages
		// 		velocity_publisher.publish(vel); // Publish the command velocity
		// 	}
		// }
		//
    // return 0;
}
