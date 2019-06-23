#!/usr/bin/env python

import argparse
import sys
import rospy
from std_msgs.msg import String
import numpy
import math
import os
 #initialize variables and node
shapes=''
c2c_distance_px=1
rospy.init_node('EnterShape')
#set publishers
pub= rospy.Publisher('Required_Shape',String, queue_size=10)
pub1= rospy.Publisher('C2C_distance_px',Int32, queue_size=10)
#C2C_distance is the distance from corner to corner

def shape():
# ask for input of shape and read it
	global shapes, c2c_distance
	shapes = raw_input("choose a formation from (line, column, triangle, diagonal, L-shape, square)")
    c2c_distance_px = raw_input("choose the corner to corner distance")
    #os.system('clear')

if __name__== '__main__':
	shape()
	while not rospy.is_shutdown():
		#print shapes
#publish the chosen shape
		pub.publish(shapes)
        pub1.publish(c2c_distance)
