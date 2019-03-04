#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from math import fabs
from nav_msgs.msg import Path
from copy import copy, deepcopy

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
        self.path = Path()
        self.pub_pose = rospy.Publisher('xbox_pose', PoseStamped, queue_size=10)
        self.pub_path = rospy.Publisher('xbox_path', Path, queue_size=10)
        self.pub_pose_next = rospy.Publisher('xbox_pose_next', PoseStamped, queue_size=10)        
        rospy.Subscriber('joy', Joy, self.callback)

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
        
        if self.button2:
            self.nextPose = deepcopy(self.currentPose)

        self.path.header.stamp = rospy.Time.now()
        self.path.header.frame_id = '/xbox'  
        self.path.poses.append(self.nextPose)

        if self.button1:
            self.path.poses = []
# def tel():
#     rospy.init_node('teleop_node')
#     Teleoperator = XboxTel()
#     r = rospy.get_param('~rate', 100)
#     rate = rospy.Rate(r)

#     while not rospy.is_shutdown():
#         if Teleoperator.lastData != None:
#             if fabs(Teleoperator.lastData.axes[1]) > 0.1:
#                 Teleoperator.currentPose.pose.position.z += Teleoperator.lastData.axes[1] / r / 2
#             if fabs(Teleoperator.lastData.axes[4]) > 0.1:
#                 Teleoperator.currentPose.pose.position.x += -Teleoperator.lastData.axes[4] / r * 1
#             if fabs(Teleoperator.lastData.axes[3]) > 0.1:
#                 Teleoperator.currentPose.pose.position.y += -Teleoperator.lastData.axes[3] / r * 1
#             if fabs(Teleoperator.lastData.axes[0]) > 0.1:
#                 Teleoperator.yaw += Teleoperator.lastData.axes[0] / r * 2
#             quaternion = tf.transformations.quaternion_from_euler(0, 0, Teleoperator.yaw)
#             Teleoperator.currentPose.pose.orientation.x = quaternion[0]
#             Teleoperator.currentPose.pose.orientation.y = quaternion[1]
#             Teleoperator.currentPose.pose.orientation.z = quaternion[2]
#             Teleoperator.currentPose.pose.orientation.w = quaternion[3]
#         Teleoperator.currentPose.header.seq += 1
#         Teleoperator.currentPose.header.stamp = rospy.Time.now()
#         Teleoperator.pub_pose.publish(Teleoperator.currentPose)
#         rate.sleep()



# if __name__ == '__main__':
#     try:
#         tel()
#     except rospy.ROSInterruptException:
#         pass