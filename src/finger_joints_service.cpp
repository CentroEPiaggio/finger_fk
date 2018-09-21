/**
    finger_joints_service.cpp

    Purpose:  a ROS node service to provide the joint states of a finger from a root link
              to a tip link

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/


//BASIC INCLUDES
#include <sstream>
#include <string>
#include <exception>
#include <math.h>

// ROS INCLUDES
#include <ros/ros.h>

// MSG INCLUDES
#include <sensor_msgs/JointState.h>

// OTHER INCLUDES
#include <cmath>

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "finger_joints_service");
	ros::NodeHandle fjs_nh;



	// Success message
	std::cout << "The Finger Joint Server ready to process requests!" << std::endl;

	// Spin
	ros::spin ();

}
