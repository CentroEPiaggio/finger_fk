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

/**********************************************************************************************
 GET FINGER JOINT STATES 
**********************************************************************************************/
void getFingerJointStates(float finger_positions_a[], float finger_positions_b[]) {
	sensor_msgs::JointState::ConstPtr finger_joint_state =
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", n);
		// ros::topic::waitForMessage<sensor_msgs::JointState>("/" + string(HAND_NAME) + "/joint_states", n);
	if (!finger_joint_state) {
		ROS_ERROR("Finger joint states not received!!! \n");
	}

	// Saving the needed joint states of finger in collision
	if (finger_name == "thumb") {
		finger_positions_a[0] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_abd_joint") - finger_joint_state->name.begin()];
		finger_positions_a[1] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint") - finger_joint_state->name.begin()];
		finger_positions_a[2] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_a[3] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint") - finger_joint_state->name.begin()];
		finger_positions_a[4] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint_mimic") - finger_joint_state->name.begin()];
	} else {
		finger_positions_b[0] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_abd_joint") - finger_joint_state->name.begin()];
		finger_positions_b[1] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint") - finger_joint_state->name.begin()];
		finger_positions_b[2] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_inner_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_b[3] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_middle_joint") - finger_joint_state->name.begin()];
		finger_positions_b[4] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_middle_joint_mimic") - finger_joint_state->name.begin()];
		finger_positions_b[5] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint") - finger_joint_state->name.begin()];
		finger_positions_b[6] =
		finger_joint_state->position[find (finger_joint_state->name.begin(),finger_joint_state->name.end(),
			string(HAND_NAME) + "_" + finger_name + "_outer_joint_mimic") - finger_joint_state->name.begin()];
	}
}

/* ******************************************************************************************** */

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
