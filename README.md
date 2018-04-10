# PISA IIT SoftHand Finger Forward Kinematics Utility

A ROS node with a service server which performs Forward Kinematics for the Pisa/IIT SoftHand. The service returns as response as a geometry_msgs::Pose the transform between the two link frames (from the ee_link to the finger_link) given in the request. The form of the service file is as follows:

string ee_link_name
string finger_link_name
float64[] joint_state_positions
---
geometry_msgs/Pose ee_frame_in_finger

## Authors

* **George Jose Pollayil** - [gpollayil](https://github.com/gpollayil)
* **Mathew Jose Pollayil** - [mpollayil](https://github.com/mpollayil)

## Getting Started

### Prerequisites

This package depends on ROS Indigo or newer and the HRL-KDL package. 

Clone this in you catkin workspace and build:
https://github.com/gt-ros-pkg/hrl-kdl.git

### Installing

To install this package just clone into your catkin_ws and catkin_make. After making please remember to make the python script file finger_fk_main.py excecutable.

## Running the finger fk server

Launch the finger_fk node using either rosrun:

```
rosrun finger_fk finger_fk_main.py
```

Or add the following to your launch file:

```
<!-- Load finger_fk service node -->
<node name="finger_forward_kinematics" pkg="finger_fk" type="finger_fk_main.py" output="screen"/>
```
