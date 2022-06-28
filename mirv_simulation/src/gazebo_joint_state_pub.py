#!/usr/bin/env python2
from __future__ import print_function
import roslib
import sys
import rospy
import numpy as np
import datetime
import time
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import message_filters
import copy
from threading import Thread, Lock
import helpful_functions_lib

# JointState message definition: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
# JointState message example:
# header: 
#   seq: 410
#   stamp: 
#     secs: 8
#     nsecs: 313000000
#   frame_id: ''
# name: [wheel_rl_joint, wheel_rr_joint, wheel_fl_joint, wheel_fr_joint]
# position: [0.0, 0.0, 0.0, 0.0]
# velocity: []
# effort: []



class gazebo_joint_state_pub:

    # Define initial/setup values
    def __init__(self):

        # # Use hardcoded parameters for debugging
        # self.gazebo_robot_prefix = 'mirv::' # gazebo prefix
        # self.base_link_name = 'base_footprint' # Name of the robot base link
        # self.base_link_offsets = [0, 0, 0, 0] # Radian offset of each wheel from the axis of the base link
        # self.axes = ['y', 'y', 'y', 'y'] # What axis the wheel turns around
        # self.link_names = ['wheel_fl', 'wheel_fr', 'wheel_rl', 'wheel_rr']
        # self.joint_names = ['wheel_fl_joint', 'wheel_fr_joint', 'wheel_rl_joint', 'wheel_rr_joint']

        # Get parameters from launch file
        self.gazebo_robot_prefix = rospy.get_param('~gazebo_prefix') # gazebo prefix
        self.base_link_name = rospy.get_param('~base_link_name') # Name of the robot base link
        self.base_link_offsets = rospy.get_param('~base_link_angle_offsets') # Radian offset of each wheel from the axis of the base link
        self.axes = rospy.get_param('~axes') # What axis the wheel turns around
        self.link_names = rospy.get_param('~link_names')
        self.joint_names = rospy.get_param('~joint_names')

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        time.sleep(10)

        # self.${publisher_name} = rospy.Publisher(${string topic_name}, ${message_type}, ${other_parameters...})
        self.joint_state_pub = rospy.Publisher('/encoder/wheel_angles', JointState, queue_size=5)
    
        # self.${subscriber_name} = rospy.Subscriber(${string topic_name}, ${message_type}, ${self.callback_function}, ${other_parameters...})
        self.gazebo_objects_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.gazebo_link_state_cb, queue_size=1)

    def gazebo_link_state_cb(self, msg):
        names = msg.name
        base_index = names.index(self.gazebo_robot_prefix + self.base_link_name)
        object_indices = []
        for name in self.link_names:
            object_indices.append(names.index(self.gazebo_robot_prefix + name))

        base_link_pose = msg.pose[base_index]
        base_link_twist = msg.twist[base_index]
        object_names = [names[i] for i in object_indices]
        object_links = [names[i] for i in object_indices]
        object_poses = [msg.pose[i] for i in object_indices]
        object_twists = [msg.twist[i] for i in object_indices]

        object_rel_angles = []
        for i, pose in enumerate(object_poses):
            angle, rel_pose = helpful_functions_lib.joint_state_from_link_poses(base_link_pose, object_poses[i], self.axes[i])
            object_rel_angles.append(angle)
            self.pub_tf(rel_pose, self.base_link_name, object_names[i])
        # This does the same thing, but is much less readable and debuggable
        # object_rel_angles = [self.joint_state_from_link_poses(base_link_pose, object_poses[i], self.axes[i]) for i in range(len(object_poses))]

        # message details:
        # name, string of model:link
        # pose, array of poses in order for each name
        # twist, array of twists ion order for each name
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.name = self.joint_names
        joint_state.position = object_rel_angles

        self.joint_state_pub.publish(joint_state)

    def pub_tf(self, pose, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w
        self.tf_broadcaster.sendTransform(t)


def main(args):
    rospy.init_node('encoder', anonymous=True)
    joint_state = gazebo_joint_state_pub()
    rospy.spin()
    # try:
    #     while True:
    #         joint_state.timer.sleep()
    #         joint_state.get_and_send_tick_count()
    #
    # except KeyboardInterrupt:
    #     print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)


# Gazebo link_states message example:
# name: ['ground_plane::link', 'mirv::base_footprint', 'mirv::wheel_fl', 'mirv::wheel_fr',
#   'mirv::wheel_rl', 'mirv::wheel_rr']
# pose:
#   -
#     position:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#       w: 1.0
#   -
#     position:
#       x: -0.478202378887
#       y: 0.103787016207
#       z: -1.57281223254e-06
#     orientation:
#       x: 2.93534792174e-06
#       y: -2.63750628713e-06
#       z: 0.477806342016
#       w: 0.878465195391
#   -
#     position:
#       x: -0.525047776522
#       y: 0.491476244632
#       z: 0.100001818832
#     orientation:
#       x: -0.0311726107596
#       y: -0.706456121563
#       z: -0.357682163298
#       w: 0.609927443802
#   -
#     position:
#       x: -0.105314064798
#       y: 0.219778002895
#       z: 0.100000447009
#     orientation:
#       x: -0.0653300523719
#       y: -0.704077784975
#       z: -0.327759779637
#       w: 0.626562035088
#   -
#     position:
#       x: -0.688070661639
#       y: 0.239636800911
#       z: 0.100000379035
#     orientation:
#       x: 0.0671504116926
#       y: -0.703913943387
#       z: -0.438877078318
#       w: 0.554421223117
#   -
#     position:
#       x: -0.268334359835
#       y: -0.0320642906587
#       z: 0.0999989205731
#     orientation:
#       x: 0.433570357946
#       y: -0.558584463244
#       z: -0.66750621611
#       w: 0.233314366441
# twist:
#   -
#     linear:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#     angular:
#       x: 0.0
#       y: 0.0
#       z: 0.0
#   -
#     linear:
#       x: -0.000315266476195
#       y: 0.000240882818652
#       z: -0.000341386909085
#     angular:
#       x: 0.00325898506982
#       y: 0.00124707949328
#       z: -0.000524881579078
#   -
#     linear:
#       x: -0.000175823810307
#       y: 0.000107888564798
#       z: -0.000388734216592
#     angular:
#       x: -0.00118812350975
#       y: -0.00151101727086
#       z: -0.000838237101907
#   -
#     linear:
#       x: -0.000208238236155
#       y: -0.000233638818881
#       z: 0.000401337480928
#     angular:
#       x: 0.00237906921295
#       y: -0.00217712003914
#       z: -0.000362531949894
#   -
#     linear:
#       x: -0.000248677206453
#       y: 7.32677328466e-05
#       z: 0.00205901523175
#     angular:
#       x: -0.000690667857166
#       y: -0.00264510365282
#       z: -0.000655687004115
#   -
#     linear:
#       x: -0.000280055291064
#       y: -0.000191753084991
#       z: -0.00141448491094
#     angular:
#       x: 0.00275699634719
#       y: 0.00342189058531
#       z: -0.000635315115488

# header:
#       seq: 0
#       stamp:
#         secs: 77
#         nsecs: 720000000
#       frame_id: "base_link"
#     child_frame_id: "wheel_fl"
#     transform:
#       translation:
#         x: 0.15
#         y: 0.25
#         z: 0.0
#       rotation:
#         x: -0.707106781185
#         y: 0.0
#         z: 0.0
#         w: 0.707106781188
#   -
#     header:
#       seq: 0
#       stamp:
#         secs: 77
#         nsecs: 720000000
#       frame_id: "base_link"
#     child_frame_id: "wheel_fr"
#     transform:
#       translation:
#         x: 0.15
#         y: -0.25
#         z: 0.0
#       rotation:
#         x: -0.707106781185
#         y: 0.0
#         z: 0.0
#         w: 0.707106781188
#   -
#     header:
#       seq: 0
#       stamp:
#         secs: 77
#         nsecs: 720000000
#       frame_id: "base_link"
#     child_frame_id: "wheel_rl"
#     transform:
#       translation:
#         x: -0.15
#         y: 0.25
#         z: 0.0
#       rotation:
#         x: -0.707106781185
#         y: 0.0
#         z: 0.0
#         w: 0.707106781188
#   -
#     header:
#       seq: 0
#       stamp:
#         secs: 77
#         nsecs: 720000000
#       frame_id: "base_link"
#     child_frame_id: "wheel_rr"
#     transform:
#       translation:
#         x: -0.15
#         y: -0.25
#         z: 0.0
#       rotation:
#         x: -0.707106781185
#         y: 0.0
#         z: 0.0
#         w: 0.707106781188
# ---
