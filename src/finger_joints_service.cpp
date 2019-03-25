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
#include <ros/subscribe_options.h>

// MSG INCLUDES
#include <sensor_msgs/JointState.h>

// SERVICE INCLUDES
#include "finger_fk/FingerJointsService.h"

// OTHER INCLUDES
#include <cmath>
#include <boost/thread.hpp>

#define DEBUG	0
#define HAND_NAME	"right_hand"

using namespace std;

// GLOBAL VARIABLES
sensor_msgs::JointState::ConstPtr full_joint_state;	// a msg where the subscriber will save the joint states

/**********************************************************************************************
 GET FINGER JOINT STATES
**********************************************************************************************/
sensor_msgs::JointState getFingerJointState(int finger_id_, bool &success_){
	// Creating a JointState to be filled up
	sensor_msgs::JointState finger_joint_state;

	// Saving the needed joint states of finger in collision
	if (finger_id_ == 1) {

		ROS_DEBUG_STREAM("STARTING TO FILL UP THUMB JOINTS!");

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

		ROS_DEBUG_STREAM("FINISHED TO FILL UP THUMB JOINTS!");
		success_ = true;

	} else if (finger_id_ == 2) {

		ROS_DEBUG_STREAM("STARTING TO FILL UP INDEX JOINTS!");

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

		ROS_DEBUG_STREAM("FINISHED TO FILL UP INDEX JOINTS!");
		success_ = true;

	} else if (finger_id_ == 3) {

		ROS_DEBUG_STREAM("STARTING TO FILL UP MIDDLE JOINTS!");

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

		ROS_DEBUG_STREAM("FINISHED TO FILL UP MIDDLE JOINTS!");
		success_ = true;

	} else if (finger_id_ == 4) {

		ROS_DEBUG_STREAM("STARTING TO FILL UP RING JOINTS!");

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

		ROS_DEBUG_STREAM("FINISHED TO FILL UP RING JOINTS!");
		success_ = true;

	} else if (finger_id_ == 5) {

		ROS_DEBUG_STREAM("STARTING TO FILL UP LITTLE JOINTS!");

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

		ROS_DEBUG_STREAM("FINISHED TO FILL UP LITTLE JOINTS!");
		success_ = true;

	} else {
		ROS_WARN("FingerJointService : something went wrong. The finger id is not correct!");
		success_ = false;
	}
	
	return finger_joint_state;
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
	ROS_DEBUG_STREAM("FILLING UP THE RESPONSE!");
	res.joint_state = finger_joints;
	ROS_DEBUG_STREAM("FINISHED FILLING UP THE RESPONSE!");

	// Ending the callback
	return success;
}

/**********************************************************************************************
 SUBSCRIBER CALLBACK
**********************************************************************************************/
void getJointStates(const sensor_msgs::JointState::ConstPtr &msg){
	// Storing the message into another global message variable
	ROS_DEBUG_STREAM("GOT JOINTSTATE MSG: STARTING TO SAVE!");
	full_joint_state = msg;
	ROS_DEBUG_STREAM("SAVED JOINTSTATE MSG!");
}



/* ******************************************************************************************** */

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "finger_joints_service");
	ros::NodeHandle* fjs_nh = new ros::NodeHandle();

	// The finger joint service server
	ros::ServiceServer fj_server = fjs_nh->advertiseService("fj_service", &getFingerJoints);

	// The subscriber for saving joint states
	ros::SubscribeOptions joint_state_so = ros::SubscribeOptions::create<sensor_msgs::JointState>("joint_states", 
		1, getJointStates, ros::VoidPtr(), fjs_nh->getCallbackQueue());
	ros::Subscriber js_sub = fjs_nh->subscribe(joint_state_so);

	// Success message
	ROS_INFO("The Finger Joint Server ready to process requests!");
	ROS_DEBUG_STREAM("DEBUG ACTIVATED!");

	// Spin
	ros::spin ();

}
