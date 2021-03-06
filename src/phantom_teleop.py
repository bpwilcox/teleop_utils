#!/usr/bin/env python

import numpy as np
import rospy
import tf

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from teleop_utils.srv import GetPose

class phantom_teleop:
    '''
    The class for receiving the Haptic device pose
    '''
    def __init__(self):
        self.vel = np.zeros((3, 1))
        self.ang_vel = np.zeros((3, 1))
        self.transform = np.eye(4)
        self.button1 = False
        self.button2 = False
        rospy.Subscriber('pose_msg', Float32MultiArray, self.callback)
        self.pub_pose = rospy.Publisher('ee_pose', PoseStamped, queue_size=10)
        self.currentPose = PoseStamped()
        self.pub_path = rospy.Publisher('ee_path', Path, queue_size=10)
        self.path = Path()
        self.service_pose = rospy.Service('teleop_pose',GetPose, self.returnPose)

    def callback(self, data_stream):
        self.transform = np.reshape(
            data_stream.data[0:16], (4, 4), order='F')
        self.hd_vel = np.asarray(data_stream.data[16:19])
        self.ang_vel = np.asarray(data_stream.data[19:22])
        if data_stream.data[22] == 1:
            self.button1 = True
        else:
            self.button1 = False
        if data_stream.data[23] == 1:
            self.button2 = True
        else:
            self.button2 = False

        self.currentPose.pose.position.x = self.transform[2,3]
        self.currentPose.pose.position.y = self.transform[0,3]
        self.currentPose.pose.position.z = self.transform[1,3]

        # R1 = tf.transformations.rotation_matrix(-np.pi, np.array([0,1,0]))
        # R2 = tf.transformations.rotation_matrix(-np.pi, np.array([1,0,0]))
        # quaternion = tf.transformations.quaternion_from_matrix(np.matmul(R2, np.matmul(R1,self.transform)))

        quaternion = tf.transformations.quaternion_from_matrix(self.transform)

        self.currentPose.pose.orientation.x = quaternion[0]
        self.currentPose.pose.orientation.y = quaternion[1]
        self.currentPose.pose.orientation.z = quaternion[2]
        self.currentPose.pose.orientation.w = quaternion[3]

        self.currentPose.header.stamp = rospy.Time.now()
        self.currentPose.header.frame_id = '/phantom'

        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = '/phantom'  
        self.path.poses.append(self.currentPose)

    def returnPose(self, resp):
        return self.currentPose

