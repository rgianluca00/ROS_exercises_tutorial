/*
 * random_controller.cpp
 * 
 * Created on: March, 2020
 *     Author: Gianluca Rinaldi
 */

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h> 
#include <math.h>
#include <time.h>

/* Global Variables */ 

bool C_obs = false;
geometry_msgs::Twist cmd_vel;
turtlesim::Pose pose;

/* Callback function for retrieving turtle position from turtlesim/Pose messages 
 * and for keeping the turtle in C_free, for us the upper side of the map (that is, y > 5.50). 
 * If y <= 5.50, turtle enters the forbidden zone (C_obs) */

void poseCallback(const turtlesim::Pose::ConstPtr& pose_msg)
{
	int SF = 3;  // Scale factor for speeding up turtle in its escape away from C_obs
	
	pose.y = pose_msg->y;
	
    std::cout << " x position is: " << pose_msg->x << " and y coordinate is: " << pose_msg->y <<  " and theta is: " << pose_msg->theta << std::endl;
	
	if (pose_msg->y <= 5.50) // Turtle is in C_obs
	{
		C_obs = true;
		
		ROS_INFO("Turtle is approaching C_obs - Move back to C_free!");
		
		/* for moving back to C_free, set yaw at zero and linear velocity with opposite sign to move from actual direction to the opposite one */
		
		cmd_vel.angular.z = 0; 
		cmd_vel.linear.x = (pose_msg->linear_velocity) * (-1 * SF);
		
	}
	else // Turtle is NOT in C_obs
	{
		C_obs = false;
	}
}

/* Main function */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_controller");
	ros::NodeHandle nh;
	
	ros::Rate rate(1);
		
	/* Publisher and Subscriber */
	
	ros::Publisher pub_vel = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
	ros::Subscriber sub_pos = nh.subscribe("/turtle1/pose", 10, &poseCallback);
	
		
	/* Control Loop */
	
    while (ros::ok()) 
    {	
		
		/* Variables declaration */
		
		float scale_factor = 0.5;
		float offset = 5.4; // [m]
		int max_vx = 3; // [m/s] -> upper bound for linear velocity
		int min_vx = -2; // [m/s] -> lower bound for linear velocity
		int max_yaw = 2; // [rad/s] -> upper bound for angular velocity
		int min_yaw = -1; // [rad/s] -> lower bound for angular velocity
		
		ros::spinOnce(); // Check turtle position in C_space and retrieve its y coordinate to built Kp
		
		float Kp = scale_factor * (pose.y - offset); // Proportional control gain with SF = 0.5 and POSITION OFFSET = 5.4, so Kp->0.1 (almost zero) if turtle apporoaches C_obs 
		
		if(!C_obs) // If turtle is in C_free then velocity control is random
		{
			srand(time(NULL)); // initialize seed for random number assignment as current time from January 1st 1970
			cmd_vel.linear.x = (rand() % (max_vx + 1) + (min_vx)) * Kp; // linear vel tends to zero when turtle approaches C_obs
			cmd_vel.angular.z = (rand() % (max_yaw + 1) + (min_yaw)) * Kp; // angular yaw tends to zero when turtle approaches C_obs
			pub_vel.publish(cmd_vel); // publish random velocity commands
		}
		else // Turtle is in C_obs 
		{
			pub_vel.publish(cmd_vel); // publish commands to move back turtle from C-obs to C_free
			sleep(1);
		}
		rate.sleep();
    }

    return 0;
}
