#!/usr/bin/env python

from finger_fk.srv import *
import roslib
import rospy
from geometry_msgs.msg import Transform, Pose, PoseStamped, Point, Point32, PointStamped, Vector3, Vector3Stamped, Quaternion, QuaternionStamped
from std_msgs.msg import Header
import tf.transformations
import tf 
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics

urdf_str = "/home/mathew/franka_ws/src/adaptive_grasp_controller/robot/panda_softhand.urdf.xacro"

def mat_to_pose(mat, transform = None):
    '''Convert a homogeneous matrix to a Pose message, optionally premultiply by a transform.
    Args:
        mat (numpy.ndarray): 4x4 array (or matrix) representing a homogenous transform.
        transform (numpy.ndarray): Optional 4x4 array representing additional transform
    Returns:
        pose (geometry_msgs.msg.Pose): Pose message representing transform.
    '''
    if transform != None:
    	mat = np.dot(transform, mat)
    pose = Pose()
    pose.position.x = mat[0,3]
    pose.position.y = mat[1,3]
    pose.position.z = mat[2,3]
    quat = tf.transformations.quaternion_from_matrix(mat)
    pose.orientation.x = quat[0]
    pose.orientation.y = quat[1]
    pose.orientation.z = quat[2]
    pose.orientation.w = quat[3]

    return pose

def finger_fk_compute(req):
	kdl_kin = KDLKinematics(robot_urdf, req.ee_link_name, req.finger_link_name)

	print "Forward Kinematics requested from %s to %s"%(req.ee_link_name, req.finger_link_name)

	joint_state_positions = req.joint_state_positions

	fing_pose = kdl_kin.forward(joint_state_positions)

	fing_pose_geom = mat_to_pose(fing_pose)

	print "Finger FK Service Response (Finger pose wrt EE) is %s"%(fing_pose)
	return FingerFkServiceResponse(fing_pose_geom)

def finger_fk_server():
    rospy.init_node('finger_fk_node')
    s = rospy.Service('finger_fk_service', FingerFkService, finger_fk_compute)
    print "Ready to compute finger FK."
    rospy.spin()

if __name__ == "__main__":
	# Getting the robot model
	robot_urdf = URDF.from_parameter_server()

	finger_fk_server()
