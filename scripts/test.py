#!/usr/bin/env python
from __future__ import division
import rospy
import math
import time
import numpy as np
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray, Int32

def test():
    rospy.init_node('formation_tester')
    pub1 = rospy.Publisher('align_rob1', Int32MultiArray, queue_size=10)
    pub2 = rospy.Publisher('align_rob2', Int32MultiArray, queue_size=10)
    pub3 = rospy.Publisher('align_rob3', Int32MultiArray, queue_size=10)
    pub4 = rospy.Publisher('align_rob4', Int32MultiArray, queue_size=10)
    pub5 = rospy.Publisher('corners_rob1', Int32MultiArray, queue_size=10)
    pub6 = rospy.Publisher('corners_rob2', Int32MultiArray, queue_size=10)
    pub7 = rospy.Publisher('corners_rob3', Int32MultiArray, queue_size=10)
    pub8 = rospy.Publisher('corners_rob4', Int32MultiArray, queue_size=10)


    test = Int32MultiArray()
    test.data = [1,1]

    pos =  Int32MultiArray()
    pos.data = [5,5]

    while not rospy.is_shutdown():
        pub1.publish(test)
        pub2.publish(test)
        pub3.publish(test)
        pub4.publish(test)

        pub5.publish(pos)
        pub6.publish(pos)
        pub7.publish(pos)
        pub8.publish(pos)


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass
