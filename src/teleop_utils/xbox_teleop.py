#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from math import fabs
from nav_msgs.msg import Path
from copy import copy, deepcopy
from teleop_utils.srv import SetPose, GetTeleop, GetTeleopResponse

class XboxTel():
    def __init__(self, rate = 100):
        x = rospy.get_param("~x", 0)
        y = rospy.get_param("~y", 0)
        z = rospy.get_param("~z", 0)
        self.r = rate  
        self.currentPose = PoseStamped()
        self.currentPose.header.seq = 0
        self.currentPose.header.stamp = rospy.Time.now()
        self.currentPose.header.frame_id = '/xbox'
        self.currentPose.pose.position.x = x
        self.currentPose.pose.position.y = y
        self.currentPose.pose.position.z = z
        self.yaw = 0
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.currentPose.pose.orientation.x = quaternion[0]
        self.currentPose.pose.orientation.y = quaternion[1]
        self.currentPose.pose.orientation.z = quaternion[2]
        self.currentPose.pose.orientation.w = quaternion[3]
        self.nextPose = deepcopy(self.currentPose)
        self.lastData = None
        self.button1 = False
        self.button2 = False
        self.button3 = False
        self.button4 = False
        self.button5 = False
        self.button6 = False
        self.button7 = False

        self.path = Path()
        self.pub_pose = rospy.Publisher('xbox_pose', PoseStamped, queue_size=10)
        self.pub_path = rospy.Publisher('xbox_path', Path, queue_size=10)
        self.pub_pose_next = rospy.Publisher('xbox_pose_next', PoseStamped, queue_size=10)        
        rospy.Subscriber('joy', Joy, self.callback)
        self.setPose_service = rospy.Service('set_xbox_pose',SetPose, self.set_pose)
        self.getTeleop_service = rospy.Service('get_xbox_teleop',GetTeleop, self.get_teleop)

    def callback(self, data):
        # self.lastData = data
        if data != None:
            # if fabs(data.axes[1]) > 0.1:
            #     self.currentPose.pose.position.z += data.axes[1] / self.r / 2
            # if fabs(data.axes[4]) > 0.1:
            #     self.currentPose.pose.position.x += -data.axes[4] / self.r * 1
            # if fabs(data.axes[3]) > 0.1:
            #     self.currentPose.pose.position.y += -data.axes[3] / self.r * 1
            # if fabs(data.axes[0]) > 0.1:
            #     self.yaw += data.axes[0] / self.r * 2

            self.currentPose.pose.position.z += data.axes[1] / self.r / 2
            self.currentPose.pose.position.x += -data.axes[4] / self.r * 1
            self.currentPose.pose.position.y += -data.axes[3] / self.r * 1
            self.yaw += data.axes[0] / self.r * 2
            quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
            self.currentPose.pose.orientation.x = quaternion[0]
            self.currentPose.pose.orientation.y = quaternion[1]
            self.currentPose.pose.orientation.z = quaternion[2]
            self.currentPose.pose.orientation.w = quaternion[3]
        self.currentPose.header.seq += 1
        self.currentPose.header.stamp = rospy.Time.now()

        if data.buttons[4] == 1:
            self.button1 = True
        else:
            self.button1 = False
        if data.buttons[5] == 1:
            self.button2 = True
        else:
            self.button2 = False

        if data.buttons[0] == 1:
            self.button3 = True
        else:
            self.button3 = False

        if data.buttons[2] == 1:
            self.button4 = True
        else:
            self.button4 = False

        if data.buttons[3] == 1:
            self.button5 = True
        else:
            self.button5 = False

        if data.buttons[6] == 1:
            self.button6 = True
        else:
            self.button6 = False
            
        if data.buttons[7] == 1:
            self.button7 = True
        else:
            self.button7 = False            

        if self.button2:
            self.nextPose = deepcopy(self.currentPose)

        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = '/xbox'  
        self.path.poses.append(self.nextPose)

        if self.button1:
            self.path.poses = []

    def set_pose(self,req):
        self.currentPose = req.pose
        return True
        
    def get_teleop(self,req):
        return self.nextPose, self.button1, self.button2, self.button3, self.button4, self.button5, self.button6, self.button7