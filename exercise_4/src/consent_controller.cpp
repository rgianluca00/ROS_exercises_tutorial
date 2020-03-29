/*
 * consent_controller.cpp
 * 
 * Created on: March, 2020
 *     Author: Gianluca Rinaldi
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include "exercise_4/RobotStatus.h"
#include <math.h>      

/* Define a struct for robot status info */

struct STATUS
{
  std::string robot_name;
  bool is_ready;
  bool is_arrived;
  float x;
  float y;
  float theta;
};

/* Gloabal Variables */

const int robot_number = 4; // total number of spawned robots
std::map<std::string, STATUS> map; // map with <KEY, MAPPED VALUE> as KEY = robot name and MAPPED VALUE as STATUS struct
std::string robot_name; 
int robot_id = 0;
bool teamReady = false;
bool teamArrived = false;
float bar[2]={0,0}; // initialize barycenter
geometry_msgs::Twist cmd_vel;

/* Publishers and Subscribers declaration */

ros::Subscriber team_status_sub;
ros::Publisher team_status_pub;
ros::Subscriber sub_pose;
ros::Publisher vel_pub;

/* Callback function for retrieving robot status info from RobotStatus msg by subscribing to /team_status topic. Then, the 
function check if the team is ready to move and if every robot has arrived at the barycenter */

void teamStatusCallback(const exercise_4::RobotStatus::ConstPtr& status_msg)
{
	int ready_robot = 0; // counter for how many robots are ready to move
	int arrived_robot = 0; // counter for how many robots have reached the target
	
	STATUS tmp_var_2;
  
	if(teamReady and teamArrived) return;
	
	// Retrieve Status message
	
	map[status_msg->robot_name].robot_name = status_msg->robot_name;
	map[status_msg->robot_name].is_ready = status_msg->is_ready;
	map[status_msg->robot_name].is_arrived = status_msg->is_arrived;
	map[status_msg->robot_name].x = status_msg->x;
	map[status_msg->robot_name].y = status_msg->y;
	
	for(auto robot : map)
	{
		tmp_var_2 = robot.second; // Second element of the pair <KEY, MAPPED VALUE> from the map is of type STATUS struct
		
		if(tmp_var_2.is_ready) 
			ready_robot++; 
		if(tmp_var_2.is_arrived) 
			arrived_robot++;
	}
	
	if(ready_robot==robot_number and teamReady==false) // enter here only once the robots are all ready to move
	{ 
	  ROS_INFO_STREAM("Team is ready. Let's move to the barycenter, "<< robot_name);
	  
	  teamReady = true;
	}
	
	if(arrived_robot==robot_number) // enter here only when the robots are all arrived to the target
	{
	  ROS_INFO_STREAM("Team is arrived. Well done, "<< robot_name);
	  
	  teamArrived = true;
	}
}

/* Callback function for retrieving turtle position from turtlesim/Pose msg */

void poseCallback(const turtlesim::Pose::ConstPtr& pose)
{
	
	map[robot_name].x = pose->x;
	map[robot_name].y = pose->y;
	map[robot_name].theta = pose->theta;
	
// 	std::cout << " x position is: " << pose->x << " and y coordinate is: " << pose->y <<  " and theta is: " << pose->theta << std::endl;

}

/* Function for publishing Status msg of type RobotStatus, that is the msg to be sent to other robots in order to compute barycenter 
 * coordinates and then reaching them once at a time. It takes 4 arguments: a bool that is true if the robot is ready to move, 
 * a bool that is true if the robot has reached the barycenter and 2 float for x and y coordinates of robot position retrieved by poseCallback */

void publishReadyStatus(bool ready, bool arrived, float x, float y) {
	
    exercise_4::RobotStatus status_msg;
	
	// Now build the Status message 

    status_msg.header.stamp = ros::Time::now();
    status_msg.robot_name = robot_name;
	status_msg.robot_id = robot_id;
    status_msg.is_ready = ready;
	status_msg.is_arrived = arrived;
	status_msg.x = x;
	status_msg.y = y;

    // Wait for the publisher to connect to subscribers, then publish the Status message
	
    sleep(1.0);
    team_status_pub.publish(status_msg);

    ROS_INFO_STREAM(robot_name << " has published its status");
}

/* Function for waiting that the other robots have sent (published) their status in order to compute barycenter and then reach it */

void waitForTeam(float x, float y)  {
	
    ros::Rate loopRate(2);
	
    // Wait until all robots are ready to move because they have all sent their status to the others
	
    while (!teamReady) {
		
        ROS_INFO_STREAM(robot_name << " is waiting for team");
		
        publishReadyStatus(true, false, x , y); // is_ready = true when the robot is ready to move
        ros::spinOnce();
        loopRate.sleep();
    }
}

/* Function that computes barycenter of the four spawned turtles as their Initial Centroid */

void compute_bar() 
{
	STATUS tmp_var_1;
	
	for(auto robot : map)
	{
		tmp_var_1 = robot.second; // Second element of the pair <KEY, MAPPED VALUE> from the map is of type STATUS struct
		
		bar[0] = bar[0] + tmp_var_1.x; // Sum x positions of the four spawned turtles
		bar[1] = bar[1] + tmp_var_1.y; // Sum y positions of the four spawned turtles
	}
	
	bar[0] = bar[0] / robot_number; // Get x coordinate of barycenter 
	bar[1] = bar[1] / robot_number; // Get y coordinate of barycenter 

	std::cout << " bar x coordinate is: " << bar[0] << " and bar y coordinate is: " << bar[1] << std::endl;

}

/* Function for checking if the previous robot is arrived at the barycenter (remember, they must reach it once at a time) */

void check_prev() 
{
	ros::Rate rate(1);
	
	std::string prev_name = "turtle" + std::to_string(robot_id); // name of previous robot in the moving-to-target chain
			
	// check if robot_name is == turtle1, if so go on -> turtle1 is the first robot to move, there is no previous robot to move before turtle1
	
	if (robot_name.compare("turtle1") == 0) 
		return; 
	
	while(ros::ok())
	{
		if(map[prev_name].is_arrived == true) // check if prevoius robot arrived, if so go on
			return;
		
		ros::spinOnce();
		rate.sleep();
	}
}

/* Function for moving to barycenter*/

void reach_bar() 
{
	float d_threshold = 0.1; // distance threshold: when distance goes below this value, target is reached
	float yaw_threshold = 0.01; // yaw threshold
	float Kp_d = 0.5; // gain for proportional control
	float Kp_yaw = 1.0; // gain for proportional control
	
	ros::Rate int_rate(10);
	
	float d = sqrt(pow(bar[0]-map[robot_name].x,2)+pow(bar[1]-map[robot_name].y,2)); // Euclidean distance between actual position and desired target (barycenter)
	float yaw = atan2((bar[1]-map[robot_name].y),(bar[0]-map[robot_name].x)) - map[robot_name].theta; // yaw from actual orientation to reach desired target
	
	/* STEP 5.1: orientate to the desired destination */
	
	while(fabs(yaw) > yaw_threshold) 
	{
		yaw = atan2((bar[1]-map[robot_name].y),(bar[0]-map[robot_name].x)) - map[robot_name].theta;
		
		cmd_vel.linear.x = 0;
		cmd_vel.angular.z = Kp_yaw * yaw; // PROPORTIONAL CONTROL
		
		std::cout << "command ang = " << cmd_vel.angular.z << " and yaw = " << yaw <<  std::endl;
		std::cout << " x position is: " << map[robot_name].x << " and y coordinate is: " << map[robot_name].y <<  " and theta is: " << map[robot_name].theta << std::endl;

		
		vel_pub.publish(cmd_vel);
		ros::spinOnce();
		int_rate.sleep();
	}
	
	/* STEP 5.2: move to the desired destination */
	
	while(fabs(d) > d_threshold) 
	{
		d = sqrt(pow(bar[0]-map[robot_name].x,2)+pow(bar[1]-map[robot_name].y,2));
		
		cmd_vel.linear.x = Kp_d * d; // PROPORTIONAL CONTROL
		cmd_vel.angular.z = 0;
		
		vel_pub.publish(cmd_vel);
		ros::spinOnce();
		int_rate.sleep();
	}

	cmd_vel.linear.x = 0;
	cmd_vel.angular.z = 0;
	vel_pub.publish(cmd_vel);
	
	ROS_INFO_STREAM("Good Job! " << robot_name << " has reached the barycenter");
}

/* Main Function */

int main(int argc, char** argv)
{
   ros::init(argc, argv, "consent_controller");
   ros::NodeHandle nh("~");
   
   ros::Rate ext_rate(100);
      
   if (nh.getParam("robot_name", robot_name))
	   ROS_INFO("Found paramter: setting robot name as %s", robot_name.c_str());
   else
	   ROS_INFO("Parameter not found, please insert it from terminal as robot_name:=<robot_id>");
   
   robot_id = (robot_name.back() - '0') - 1; // update robot id by taking the last character of the string robot_name ("turtle<id>") and subtract 1
   
    /* Publishers and Subscribers */
   
   team_status_pub = nh.advertise<exercise_4::RobotStatus>("/team_status", 10);
   team_status_sub = nh.subscribe("/team_status", 20, &teamStatusCallback);
   
   vel_pub = nh.advertise<geometry_msgs::Twist>("/" + robot_name + "/cmd_vel", 10); 
   sub_pose = nh.subscribe("/" + robot_name + "/pose", 10, &poseCallback); 
   
   // wait for pub-sub
   
   sleep(2); 
   
   ros::spinOnce(); // first callbacks spin
   
   sleep(1); 
   
   /* MAIN PROCEDURE -------------------------------------------------------------------------------------------------------*/
   
   // STEP 1: Publish robot status message to other robots
   
   publishReadyStatus(true, false, map[robot_name].x, map[robot_name].y); 
   
   // STEP 2: Wait until all the robots have sent and recieved the message. When that happens, the team is finally ready to move

   waitForTeam(map[robot_name].x, map[robot_name].y); 
   
   // STEP 3: Compute barycenter with the initial positions, that is my destination goal 
   
   compute_bar(); 
   
   // STEP 4: Check "arrived" status of the previous robot -> only one robot at a time can move to the goal
   
   check_prev(); 
   
   // STEP 5: Go and reach barycenter of the four spawned robots
   
   reach_bar(); 
   
   // STEP 6: Publish again robot status to confirm robot has reached the goal ("is arrived" now equals to true)
   
   publishReadyStatus(true, true, map[robot_name].x, map[robot_name].y); 
   
   /* --------------------------------------------------------------------------------------------------------------------- */
   
   ros::spin();

   return 0;
}
