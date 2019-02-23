#!/usr/bin/env python

import rospy

from std_msgs.msg import Float32MultiArray
from phantom_teleop import phantom_teleop

def teleop():
    rospy.init_node('teleop_node')
    Teleoperator = phantom_teleop()
    R = rospy.get_param('~rate', 5)

    rate = rospy.Rate(R)

    while not rospy.is_shutdown():
        Teleoperator.pub_pose.publish(Teleoperator.currentPose)
        Teleoperator.pub_path.publish(Teleoperator.path)
        rate.sleep()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException:
        pass