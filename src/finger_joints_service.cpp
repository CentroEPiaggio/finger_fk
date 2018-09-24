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

// SERVICE INCLUDES
#include "finger_fk/FingerJointsService.h"

// OTHER INCLUDES
#include <cmath>

#define HAND_NAME		"right_hand"

using namespace std;

// GLOBAL VARIABLES
ros::NodeHandle* fjs_nh_ptr; 			// a pointer to the node handle for the callback

/**********************************************************************************************
 GET FINGER JOINT STATES
**********************************************************************************************/
sensor_msgs::JointState getFingerJointState(int finger_id_, bool &success_){
	// Listening the whole joint state
	sensor_msgs::JointState::ConstPtr full_joint_state =
		ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states", *fjs_nh_ptr);
	if (!full_joint_state) {
		ROS_ERROR("Finger joint states not received!!! \n");
		success_ = false;
	}

	// Creating a JointState to be filled up
	sensor_msgs::JointState finger_joint_state;

	// Saving the needed joint states of finger in collision
	if (finger_id_ == 1) {

		int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_thumb_abd_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_thumb_inner_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_thumb_inner_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_thumb_outer_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_thumb_outer_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

	} else if (finger_id_ == 2) {

		int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_abd_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_inner_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_inner_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_middle_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_middle_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_outer_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_index_outer_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

	} else if (finger_id_ == 3) {

		int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_abd_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_inner_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_inner_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_middle_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_middle_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_outer_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_middle_outer_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

	} else if (finger_id_ == 4) {

		int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_abd_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_inner_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_inner_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_middle_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_middle_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_outer_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_ring_outer_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

	} else if (finger_id_ == 5) {

		int index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_abd_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_inner_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_inner_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_middle_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_middle_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_outer_joint") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

		index = find (full_joint_state->name.begin(),full_joint_state->name.end(),
			string(HAND_NAME) + "_little_outer_joint_mimic") - full_joint_state->name.begin();
		finger_joint_state.name.push_back(full_joint_state->name[index]);
		finger_joint_state.position.push_back(full_joint_state->position[index]);

	} else {
		ROS_WARN("The Finger Joint Server ready to process requests!");
		success_ = false;
	}
	success_ = true;
}

/**********************************************************************************************
 SERVICE CALLBACK
**********************************************************************************************/
bool getFingerJoints(finger_fk::FingerJointsService::Request &req,
	finger_fk::FingerJointsService::Response &res){
		int finger_id = req.finger_id;
		bool success;

		// Getting the finger joint state from palm_link
		sensor_msgs::JointState finger_joints = getFingerJointState(finger_id, success);

		// Filling up the Response
		res.joint_state = finger_joints;

		// Ending the callback
		return success;
	}



/* ******************************************************************************************** */

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "finger_joints_service");
	ros::NodeHandle fjs_nh;
	fjs_nh_ptr = &fjs_nh;													// initializing the pointer

	// The finger joint service server
	ros::ServiceServer fj_server = fjs_nh.advertiseService("fj_service", &getFingerJoints);

	// Success message
	ROS_INFO("The Finger Joint Server ready to process requests!");

	// Spin
	ros::spin ();

}
