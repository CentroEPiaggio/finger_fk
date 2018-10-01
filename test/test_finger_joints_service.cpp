/**
    test_finger_joints_service.cpp

    Purpose:  a ROS node to test the service "finger_joints_service"

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

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "test_finger_joints_service");

  // Looking if correct number of arguments
  if (argc != 2) {
    ROS_INFO("Usage: This test client requires an Int input!");
    return 1;
  }

	ros::NodeHandle test_fjs_nh;

	// The finger joint service client
	ros::ServiceClient fj_client = test_fjs_nh.serviceClient<finger_fk::FingerJointsService>("fj_service");

  // Creating an srv with input arg and filling up
  ROS_INFO("The Finger Joint srv is being filled!");
  finger_fk::FingerJointsService srv;
  srv.request.finger_id = atoll(argv[1]);

  // Calling the service
  if (fj_client.call(srv)) {
    ROS_INFO("The result is as follows:");
    for(size_t i = 0; i < srv.response.joint_state.name.size(); i++) {
      cout << i << ", " << srv.response.joint_state.name[i] << " : " <<
        srv.response.joint_state.position[i] << ";" << endl;
    }
  } else {
    ROS_ERROR("Failed to call finger_joints_service");
    return 1;
  }

  // Success
	ROS_INFO("SUCCESSSS!");

  return 0;
}
