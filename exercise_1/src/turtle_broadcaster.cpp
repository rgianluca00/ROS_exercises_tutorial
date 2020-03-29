/*
 * turtle_broadcaster.cpp
 * 
 * Created on: March, 2020
 *     Author: Gianluca Rinaldi
 */


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>

/* Callback function for retrieving 2D turtle pose
 * to be broadcast to tf */

void poseCallback(const turtlesim::PoseConstPtr& msg){
	
  static tf::TransformBroadcaster br; // Create TransformBroadcaster object
  
  /* Create Transform object for transforming 2D pose into 3D */
  
  tf::Transform transform; 
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) ); 
  tf::Quaternion q;
  q.setRPY(0, 0, msg->theta); // yaw = pose.theta
  
  transform.setRotation(q);
  
  /* Sending transform with a time stamp, a parent (world) and a child (turtle 1) */
  
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle1"));
  
}

/* Main function */

int main(int argc, char** argv){
	
  ros::init(argc, argv, "turtle_broadcaster");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/turtle1/pose", 10, &poseCallback); // Subscriber to "/pose" topic, it retrives turtle1 pose

  ros::spin();
  
  return 0;
};
