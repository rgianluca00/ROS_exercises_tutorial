/*
 * obstacle_controller.cpp
 * 
 * Created on: March, 2020
 *     Author: Gianluca Rinaldi
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

/* Define a struct to use it as a two element vector */

struct VECTOR 
{
	float x;
	float y;
};

/* Define a struct for robot pose */

struct POSE 
{
	VECTOR pose;
	float theta;
	float linear_velocity;
	float angular_velocity;
};

/* Global Variables */

const int number_of_obs = 2; // total number of obstacles (it is constant)
POSE robot_pose; // struct POSE for robot pose
VECTOR obs[number_of_obs]; // 2-element struct vector for the two obstacles: obs[0] refers to obstacle1, obs[1] refers to obstacle2
int obsPose_counter = 0; // counter for how many obstacle position are being sent
geometry_msgs::Twist cmd_vel;

/* Publisher and Subscurber declaration */

ros::Publisher vel_pub;
ros::Subscriber sub_pos;
ros::Subscriber sub_obs1;
ros::Subscriber sub_obs2;

/* Callback function for retrieving turtle1, obstacle1 and obstacle2 positions from turtlesim/Pose msg 
 * and for updating counter for sent obstacles positions */

void poseCallback(const turtlesim::Pose::ConstPtr& pose, int& arg) 
{
	if (arg == 0) // arg = 0 indicates turtle1
	{
		robot_pose.pose.x = pose->x;
		robot_pose.pose.y = pose->y;
		robot_pose.theta = pose->theta;
	}
	else // arg = 1 indicates obstacle1, arg = 2 indicates obstacle2
	{
		obs[arg-1].x = pose->x; 
		obs[arg-1].y = pose->y;
		obsPose_counter++; // increment counter each time an obstacle position is recieved
		
// 		ROS_INFO_STREAM("BEWARE! There is an obstacle in x= " << obs[arg-1].x << " and y= " << obs[arg-1].y);
	}
}

/* Function for waiting that all the obstacle position are sent by updating the obsPose_counter in the poseCallback */

void waitForObs() 
{
	ros::Rate wait_rate(100);
	
	while(obsPose_counter < number_of_obs) 
	{
		ros::spinOnce(); // update counter in the poseCallback
		wait_rate.sleep();
	}
	
	obsPose_counter = 0; // reset counter
}

/* Function that computes attractive and repulsive forces according to the "Artificial potential based planning" theory. 
 * Attractive forces are computed so that the robot turtle1 can tend to the goal configuration q_g, while repulsive forces 
 * are computed so that turtle1 can move in C_free whithout any collisions with convex components of C_obs, in this case the other 
 * two turtles, obstacle1 and obstacle2. Repulsive forces must not be invasive, so they will consider obstacle radius of influence and 
 * distance from the robot in their computation. This function returns a struct VECTOR element as toal force (attractive + repulsive) element */

VECTOR compute_forces(VECTOR* q_g) 
{
	float Ka = 1; // attractive gain
	float Kr = 4; // repulsive gain
	float radius_of_obs = 3; // radius of influence 
	float distance_from_obs = 0.0; // euclidean distance between the robot and the obstacle

	VECTOR error_q = {0.0, 0.0}; // distance error between desired configuration (the goal q_g) and actual position
	VECTOR repulsive_f = {0.0, 0.0}; // Repulsive Force vector
	VECTOR total_f = {0.0, 0.0}; // Total Force = Attractive + sum(Repulsive)
	
	/* Compute Attractive Force related to the one goal q_g */
	
	error_q.x = q_g->x - robot_pose.pose.x;
	error_q.y = q_g->y - robot_pose.pose.y;
	
	total_f.x = Ka * error_q.x;
	total_f.y = Ka * error_q.y;
	
	/* Compute Repulsive Forces related to the two obstacles */
	
	for(int i = 0; i < number_of_obs; i++) // 1 repulsive force computed for 1 obstacles, here we have 2 obstacles so 2 repulsive forces
	{
		distance_from_obs = sqrt(pow(obs[i].x - robot_pose.pose.x,2) + pow(obs[i].y - robot_pose.pose.y,2));
		
		// N.B. The Repulsive Forces must not be invasive, so they enter in action only inside their radius of influence, otherwise they equals zero
		
		if (distance_from_obs <= radius_of_obs) 
		{
			// Set gamma = 2 from the original formula
			
			repulsive_f.x = ((Kr/pow(distance_from_obs,2)) * ((1/distance_from_obs) - (1/radius_of_obs))) * (robot_pose.pose.x - obs[i].x);
			repulsive_f.y = ((Kr/pow(distance_from_obs,2)) * ((1/distance_from_obs) - (1/radius_of_obs))) * (robot_pose.pose.y - obs[i].y);
		}
		else // if the robot is far from the obstacle, repulsive force is zero
		{
			repulsive_f.x = 0.0;
			repulsive_f.y = 0.0;
		}
		
		/* Compute Total Force by summing the attractive force and the sum of the two repulsive forces (one for each obstacle) */
		
		total_f.x += repulsive_f.x;
		total_f.y += repulsive_f.y;
	}
	
	std::cout << "total force is (total_f_x: = " << total_f.x << ", total_f_y: = " << total_f.y << ")" << std::endl;
	
	return total_f; // return total force vector
}

/* Function for reorienting theta (robot orientation) by taking total force angle into account */

void reorient_theta(float yaw_t, float yaw_gain, VECTOR* force) 
{
	ros::Rate reorient_rate(200);
	
	float f_yaw = atan2(force->y,force->x) - robot_pose.theta;
		
	while(fabs(f_yaw) > yaw_t) 
	{
		f_yaw = atan2(force->y,force->x) - robot_pose.theta; // reorient yaw basing on total force angle and actual theta
	
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = yaw_gain * f_yaw; // PROPORTIONAL CONTROL
				
		vel_pub.publish(cmd_vel);
		ros::spinOnce();
		reorient_rate.sleep();
	}
}

/* Function for reaching the desired goal configuration q_g by taking into account attractive and repulsive forces */

void reach_qg(VECTOR* q_g) 
{
	ros::Rate reach_rate(100);
	
	float d_threshold = 0.3; // distance threshold: when distance goes below this value, target q_g is reached
	float yaw_threshold = 0.03; // yaw threshold for reorienting the robot 
	float Kp_f = 0.9; // gain for proportional force control
	float Kp_yaw = 1.1; // gain for proportional yaw control
	
	VECTOR f;
	
	float total_f_magn; // total force vector magnitude  
	float d = sqrt(pow(q_g->x-robot_pose.pose.x,2)+pow(q_g->y-robot_pose.pose.y,2)); // Euclidean distance between actual position and desired target (q_g)

	/* Move to the desired destination by taking total force into account */
	
	while(d > d_threshold) 
	{
		f = compute_forces(q_g); // compute total force depending on the goal configuration
		
		/* Orientate theta to the desired destination by taking total force into account */
		
		reorient_theta(yaw_threshold, Kp_yaw, &f);
		
		/* Compute total force vector magnitude and move to the goal with a linear velocity command */
		
		total_f_magn = sqrt(pow(f.x,2)+pow(f.y,2));
		
		cmd_vel.linear.x = Kp_f * total_f_magn; // PROPORTIONAL CONTROL
		cmd_vel.angular.z = 0;
		
		vel_pub.publish(cmd_vel);
		
		d = sqrt(pow(q_g->x-robot_pose.pose.x,2)+pow(q_g->y-robot_pose.pose.y,2));

		ros::spinOnce();
		reach_rate.sleep();
	}
	
	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	vel_pub.publish(cmd_vel);
	
	ROS_INFO_STREAM("Good Job! turtle1 has reached its goal q_g");
}

/* Main Function */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_controller");
    ros::NodeHandle nh("~");
    ros::Rate ext_rate(100);
	
	VECTOR q_g = {0,0}; // q_g is the GOAL configuration for the robot turtle1, it is of type struct VECTOR
	
	if (nh.getParam("goal_x_coord", q_g.x))
	{
	   ROS_INFO("Found paramter: setting goal x configuration as %f", q_g.x);
	}
	else
	{
		ROS_INFO("Parameter not found: please set it in the launch file as <param name = \"goal_x_coord\" value = \"<desired_goal_x>\"/>");
	}
	
   if (nh.getParam("goal_y_coord", q_g.y))
	{
	   ROS_INFO("Found paramter: setting goal y configuration as %f", q_g.y);
	}
	else
	{
		ROS_INFO("Parameter not found: please set it in the launch file as <param name = \"goal_y_coord\" value = \"<desired_goal_y>\"/>");
	}
	
	vel_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
   
    sub_pos = nh.subscribe<turtlesim::Pose>("/turtle1/pose", 10, boost::bind(poseCallback, _1, 0)); // arg = 0 of boost::bind for turtle1 position
	sub_obs1 = nh.subscribe<turtlesim::Pose>("/obstacle1/pose", 10, boost::bind(poseCallback, _1, 1)); // arg = 1 of boost::bind for obstacle1 position
	sub_obs2 = nh.subscribe<turtlesim::Pose>("/obstacle2/pose", 10, boost::bind(poseCallback, _1, 2)); // arg = 2 of boost::bind for obstacle2 position

	// wait for pub-sub
	
	sleep(2); 
	
	waitForObs(); // wait that all the obstacle position are sent
	
	reach_qg(&q_g); // reach q_g that is the desired configuration (my goal)
	
    ros::spin();

    return 0;
}
