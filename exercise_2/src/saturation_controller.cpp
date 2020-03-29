/*
 * saturation_controller.cpp
 * 
 * Created on: March, 2020
 *     Author: Gianluca Rinaldi
 */

#include "ros/ros.h"
#include <iostream>
#include <exercise_2/CustomVelMsg.h>
#include <geometry_msgs/Twist.h>

/* Global variables */

float vel; // linear_velocity
float ang; // angular_velocity
bool command_ready = false;

/* Function for unpacking custom control velocity command message */

void ControlMsgCallback(const exercise_2::CustomVelMsg::ConstPtr& ctrl_msg) 
{
	vel = ctrl_msg->linear_velocity;
	ang = ctrl_msg->angular_yaw;

	command_ready = true; // set command_ready as true because command message is arrived
}

/* Function for applying saturation on the command velocity control */

float saturate(float min_limit, float max_limit, float input) 
{
	if (input < min_limit) // if min limit is exceeded return min limit as command velocity control
	{
		ROS_INFO("Current value is exceeding minimum limit");
		return min_limit;
	}
	else if (input > max_limit) // if max limit is exceeded return max limit as command velocity control
	{
		ROS_INFO("Current value is exceeding maximum limit");
		return max_limit;
	}
	
	return input;
}


/* Main function */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "saturation_controller");
	ros::NodeHandle nh("~");
	
	float min_linear; // saturation limit for robot minimum linear velocity
	float max_linear; // saturation limit for robot max linear velocity
	float min_angular; // saturation limit for robot min angualr velocity
	float max_angular; // saturation limit for robot max angular velocity
		
	/* Get saturation limits from launch or set them to default value */
	
	if (nh.getParam("min_linear", min_linear)) 
		ROS_INFO("Found parameter: setting saturation limit for robot min linear velocity at: %f", min_linear);
	else 
	{
		min_linear = -2.0;
		ROS_INFO("Parameter not found: setting saturation limit for robot min linear velocity at: %f", min_linear);

	}
	
	if (nh.getParam("max_linear", max_linear)) 
		ROS_INFO("Found parameter: setting saturation limit for robot max linear velocity at: %f", max_linear);
	else 
	{
		max_linear = 2.0;
		ROS_INFO("Parameter not found: setting saturation limit for robot max linear velocity at: %f", max_linear);

	}
	
	if (nh.getParam("min_angular", min_angular)) 
		ROS_INFO("Found parameter: setting saturation limit for robot min angualr velocity at: %f", min_angular);
	else 
	{
		min_angular = -2.0;
		ROS_INFO("Parameter not found: setting saturation limit for robot min angular velocity at: %f", min_angular);

	}
	
	if (nh.getParam("max_angular", max_angular)) 
		ROS_INFO("Found parameter: setting saturation limit for robot max angular velocity at: %f", max_angular);
	else 
	{
		max_angular = 2.0;
		ROS_INFO("Parameter not found: setting saturation limit for robot max angular velocity at: %f", max_angular);

	}
	
	ros::Rate rate(100);
	
	/* Publisher and Subscriber */
	
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",1000);
	
	ros::Subscriber control_sub = nh.subscribe("/custom_vel_control",1000, &ControlMsgCallback);
	
	/* Control Loop */
	
    while(ros::ok()) // Keep spinning loop until user presses Ctrl+C
    {
		geometry_msgs::Twist cmd_vel; // velocity command to be published (with saturation applied)
		
		if(command_ready) // if command is arrived
		{
			cmd_vel.linear.x = saturate(min_linear, max_linear, vel); // apply saturation on the linear velocity along x-axis
			cmd_vel.angular.z = saturate(min_angular, max_angular, ang); // apply saturation on the angular velocity around z-axis (yaw rate)

			vel_pub.publish(cmd_vel); // publish desired velocity values
		
			command_ready = false;
		}
		
		ros::spinOnce();
		rate.sleep();
    }

    return 0;
}
