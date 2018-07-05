/**
    main.cpp

    Purpose: a ROS node to publish the joints of the PISA/IIT SoftHand from the synergy_joint read from the robot

    Input Topic: 	/soft_hand/joint_states
    Output Topic: 	/all_hand_joint_states

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

#define DEBUG			1										// if DEBUG 1 prints additional couts
#define HAND_PREFIX		"right_hand_"							// Prefix of the Soft_Hand

// OTHER CONSTANT
#define SCALE			0.75									// Scale for computing the joints of the fingers

#define abd_lb  		(-30.0)
#define abd_ub   		30.0
#define thumb_abd_lb   	0.0
#define thumb_abd_ub   	90.0
#define inner_lb   		0.0
#define inner_ub   		45.0
#define middle_lb   	0.0
#define middle_ub   	45.0
#define outer_lb   		0.0
#define outer_ub   		45.0
#define pi 		   		3.1415926535897931
// #define velocity   100.0
// #define effort   100.0
// #define damping   0.0
// #define friction   0.0

using namespace std;

// GLOBAL VARIABLES
ros::Publisher pub_fing_joint_states; 							// publisher for all the joint states of the finger

// CALLBACK FUNCTION
void publish_fing_joint_states(const sensor_msgs::JointState::ConstPtr& input_joint_msg){
	sensor_msgs::JointState syn_msg = *input_joint_msg;

	// Creating the whole joint state msg
	sensor_msgs::JointState whole_joint_msg;

	// Filling up the whole joint state msg
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "synergy_joint");									// synergy_joint
	whole_joint_msg.position.push_back(syn_msg.position[find (syn_msg.name.begin(),syn_msg.name.end(), 
			string(HAND_PREFIX) + "synergy_joint") - syn_msg.name.begin()]);

	// THUMB
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "thumb_abd_joint");									// thumb_abd_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.3 * whole_joint_msg.position[0] *(thumb_abd_ub - thumb_abd_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "thumb_inner_joint");									// thumb_inner_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.3 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "thumb_inner_joint_mimic");								// thumb_inner_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.3 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "thumb_outer_joint");									// thumb_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "thumb_outer_joint_mimic");								// thumb_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	// INDEX
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_abd_joint");										// index_abd_joint
	whole_joint_msg.position.push_back(float (SCALE) * 0.2 * whole_joint_msg.position[0]);
 
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_inner_joint");									// index_inner_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_inner_joint_mimic");							// index_inner_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_middle_joint");									// index_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_middle_joint_mimic");							// index_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_outer_joint");									// index_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "index_outer_joint_mimic");								// index_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	// MIDDLE
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_abd_joint");									// middle_abd_joint
	whole_joint_msg.position.push_back(float (SCALE) * 0.0 * whole_joint_msg.position[0]);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_inner_joint");									// middle_inner_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_inner_joint_mimic");							// middle_inner_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_middle_joint");									// middle_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_middle_joint_mimic");							// middle_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_outer_joint");									// middle_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "middle_outer_joint_mimic");							// middle_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	// RING
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_abd_joint");										// ring_abd_joint
	whole_joint_msg.position.push_back(float (SCALE) * 0.2 * whole_joint_msg.position[0]);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_inner_joint");									// ring_inner_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_inner_joint_mimic");								// ring_inner_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_middle_joint");									// ring_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_middle_joint_mimic");								// ring_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_outer_joint");									// ring_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "ring_outer_joint_mimic");								// ring_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	// LITTLE
	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_abd_joint");									// little_abd_joint
	whole_joint_msg.position.push_back(float (SCALE) * 0.4 * whole_joint_msg.position[0]);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_inner_joint");									// little_inner_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_inner_joint_mimic");							// little_inner_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(inner_ub - inner_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_middle_joint");									// little_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_middle_joint_mimic");							// little_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(middle_ub - middle_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_outer_joint");									// little_outer_joint
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	whole_joint_msg.name.push_back(string(HAND_PREFIX) + "little_outer_joint_mimic");							// little_outer_joint_mimic
	whole_joint_msg.position.push_back(float (SCALE) * (1.0 * whole_joint_msg.position[0] *(outer_ub - outer_lb))*pi/180);

	pub_fing_joint_states.publish(whole_joint_msg);
	
}

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "finger_joints_pub");
	ros::NodeHandle fjp_nh;

	// Creating a ROS subscriber for the input synergy joint state
	ros::Subscriber sub = fjp_nh.subscribe("input_topic", 1, publish_fing_joint_states);

	// Creating a ROS publisher for publishing all the joint states of the finger
	pub_fing_joint_states = fjp_nh.advertise<sensor_msgs::JointState>("output_topic", 1);

	// Success message
	std::cout << "All the joint states of the SoftHand are being published from input_topic to output_topic!" << std::endl;

	// Spin
	ros::spin ();

}