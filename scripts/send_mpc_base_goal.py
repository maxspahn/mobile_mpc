#!/usr/bin/env python

import rospy
import time
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray


def sendSimpleGoal():
    rospy.init_node("mpcGoalPublisher_boxer")
    pub = rospy.Publisher("/boxer/mpc_goal", Float64MultiArray, queue_size=10)
    f = Float64MultiArray()
    f.data = [12.0, 4.0, 0.0]
    pub.publish(f)


if __name__ == "__main__":
    while not rospy.is_shutdown():
        sendSimpleGoal()
        time.sleep(5)
