import roslib
import sys
import rospy
import numpy as np
import datetime
import time
from geometry_msgs.msg import Pose
from dse_msgs.msg import PoseMarkers
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
import tf2_ros
import tf2_geometry_msgs

import constants


# From https://answers.ros.org/question/323075/transform-the-coordinate-frame-of-a-pose-from-one-fixed-frame-to-another/
def transform_pose(input_pose, from_frame, to_frame):
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def joint_state_from_link_poses(pose1, pose2, axis):
    # print('computing joint state')

    delta = get_frame_transform(pose2, pose1)
    delta_euler = np.array(quat_from_pose2eul(delta.orientation))

    if axis[0] == '-':
        delta_euler = -delta_euler

    axis_index = ['z', 'y', 'x'].index(axis)
    additional_sign = np.sign(delta_euler[(axis_index + 1) % 3])
    angle_rad = delta_euler[axis_index] * additional_sign

    return angle_rad, delta

    # given two poses and an axis, compute the difference between the poses along the specified axis.
    # Need some way to make sure we don't have angle flipping issues


# Compute the 2D rotation matrix from the angle theta
def theta_2_rotm(theta):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return R


# Compute the 2D rotation matrix from the angle theta
def rotm_2_theta(R):
    theta = np.arctan2(-R[0, 1], R[0, 0])
    return theta


# Converts a quaternion into euler angles, using the euler order described in constants.py
def quat2eul(quat):
    r = R.from_quat(quat)
    eul = r.as_euler(constants.EULER_ORDER)
    return eul


# Converts euler angles into a quaternion, using the euler order described in constants.py
def eul2quat(eul):
    r = R.from_euler(constants.EULER_ORDER, eul[:, 0])
    quat = r.as_quat()
    return quat


# Expects a quaternion in the form: orientation.x,y,z,w
def quat_from_pose2eul(orientation):
    quat = [0, 0, 0, 0]
    quat[0] = orientation.x
    quat[1] = orientation.y
    quat[2] = orientation.z
    quat[3] = orientation.w
    eul = quat2eul(quat)
    return eul


# Expects a quaternion in the form: orientation.x,y,z,w
def euler2quat_from_pose(orientation, euler):
    quat = eul2quat(euler)
    orientation.x = quat[0]
    orientation.y = quat[1]
    orientation.z = quat[2]
    orientation.w = quat[3]
    return orientation


# Grab the relevant chunk from the input matrix
def sub_matrix(matrix, ids, id, size):
    i = np.where(ids == id)[0][0]
    i_min = i * size
    i_max = i_min + size
    return matrix[i_min:i_max, i_min:i_max]


# Fill in a multi-array ROS message type with a 2D input array
def multi_array_2d_input(mat, multi_arr):
    multi_arr.layout.dim.append(MultiArrayDimension())
    multi_arr.layout.dim.append(MultiArrayDimension())
    multi_arr.layout.dim[0].label = 'rows'
    multi_arr.layout.dim[0].size = np.shape(mat)[0]
    multi_arr.layout.dim[0].stride = np.shape(mat)[0]*np.shape(mat)[1]
    multi_arr.layout.dim[1].label = 'cols'
    multi_arr.layout.dim[1].size = np.shape(mat)[1]
    multi_arr.layout.dim[1].stride = np.shape(mat)[1]
    multi_arr.layout.data_offset = 0

    multi_arr.data = mat.flatten()
    return multi_arr


# Grab and return a 2D array from a multi-array ROS message
def multi_array_2d_output(multi_arr):
    arr = np.array(multi_arr.data)
    shape = [multi_arr.layout.dim[0].size, multi_arr.layout.dim[1].size]
    mat = arr.reshape(shape)
    return mat


def get_frame_transform(from_frame, to_frame):
    position = [0, 0, 0]
    position[0] = from_frame.position.x
    position[1] = from_frame.position.y
    position[2] = from_frame.position.z
    quat = [0, 0, 0, 0]
    quat[0] = from_frame.orientation.x
    quat[1] = from_frame.orientation.y
    quat[2] = from_frame.orientation.z
    quat[3] = from_frame.orientation.w
    t1 = position
    r1 = R.from_quat(quat)
    R1 = r1.as_dcm()

    position = [0, 0, 0]
    position[0] = to_frame.position.x
    position[1] = to_frame.position.y
    position[2] = to_frame.position.z
    quat = [0, 0, 0, 0]
    quat[0] = to_frame.orientation.x
    quat[1] = to_frame.orientation.y
    quat[2] = to_frame.orientation.z
    quat[3] = to_frame.orientation.w
    t2 = position
    r2 = R.from_quat(quat)
    R2 = r2.as_dcm()

    tz = (np.transpose(R1).dot(t2) - np.transpose(R1).dot(t1))
    Rz = np.transpose(R1).dot(R2)
    rz = R.from_dcm(Rz)
    rz = rz.as_quat()

    result = Pose()
    result.position.x = tz[0]
    result.position.y = tz[1]
    result.position.z = tz[2]
    result.orientation.x = rz[0]
    result.orientation.y = rz[1]
    result.orientation.z = rz[2]
    result.orientation.w = rz[3]
    return result
