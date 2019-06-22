#!/usr/bin/env python
from __future__ import division
import rospy
import math
import time
import numpy as np
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray, Int32

"""
  ** GLOBAL KNOWLEDGE NODE FOR FORMATION **
  The node acts as a centralized host for variables
  that must be shared across all robots in the system.

  The node is currently optimized for 4 Robots in a system
"""

# Global Knowledge Variables
# FIRST: Parameters that will be later modified by robots
align_axis = [ [0,0],[0,0],[0,0],[0,0] ]
                                            # A list of lists. Each sublist for index i indicates which
                                            # axis in the i th robot is already aligned
                                            # This list is flattened and to be reshaped when recieved.

shape_corner_robots = [ [0,0],[0,0],[0,0],[0,0] ]
                                            # A list of lists. Each sublist with index i indicates
                                            # the coordinates of the robot that make the shape corners
                                            # This list is flattened and to be reshaped when recieved.

# SECOND: Parameters that are modified from the Pattern Estimation Layer
shape_length = 0                            # Integer for the required shape side length
shape_sides = 0                             # Integer value that indicates no. of shape sides
x_sides = [0,0,0,0]                         # List of integers that indicates saturation Value for X Sides
y_sides = [0,0,0,0]                         # List of integers that indicates saturation value for Y Sides

#Editing Align Axis Callback Functions (1 for each robot)
def edit_align_rob1_callback(data):
    global align_axis
    align_axis[0][0] = data.data[0]
    align_axis[0][1] = data.data[1]

def edit_align_rob2_callback(data):
    global align_axis
    align_axis[1][0] = data.data[0]
    align_axis[1][1] = data.data[1]

def edit_align_rob3_callback(data):
    global align_axis
    align_axis[2][0] = data.data[0]
    align_axis[2][1] = data.data[1]

def edit_align_rob4_callback(data):
    global align_axis
    align_axis[3][0] = data.data[0]
    align_axis[3][1] = data.data[1]

#Editing Shape Corners Array Callback Functions (1 for each robot)
def edit_corners_rob1_callback(data):
    global shape_corner_robots
    shape_corner_robots[0][0] = data.data[0]
    shape_corner_robots[0][1] = data.data[1]

def edit_corners_rob2_callback(data):
    global shape_corner_robots
    shape_corner_robots[1][0] = data.data[0]
    shape_corner_robots[1][1] = data.data[1]

def edit_corners_rob3_callback(data):
    global shape_corner_robots
    shape_corner_robots[2][0] = data.data[0]
    shape_corner_robots[2][1] = data.data[1]

def edit_corners_rob4_callback(data):
    global shape_corner_robots
    shape_corner_robots[3][0] = data.data[0]
    shape_corner_robots[3][1] = data.data[1]

def shape_setter():
    global shape_length, shape_sides, x_sides, y_sides
    # Predefining some shape parameters as if
    # pattern estimation layer is inputting them
    shape_length = 70       # Side length in cm
    shape_sides = 4         # 4 sides for square formation
    x_sides=[0,0,0,0]     # Zero x saturation values (for only 4 robots)
    y_sides=[0,0,0,0]     # Zero y saturation values (for only 4 robots)

def global_node_listener():
    #Align Axis Edit Requests
    rospy.Subscriber('align_rob1', Int32MultiArray, edit_align_rob1_callback)
    rospy.Subscriber('align_rob2', Int32MultiArray, edit_align_rob2_callback)
    rospy.Subscriber('align_rob3', Int32MultiArray, edit_align_rob3_callback)
    rospy.Subscriber('align_rob4', Int32MultiArray, edit_align_rob4_callback)

    #Shape Corners Callbacks
    rospy.Subscriber('corners_rob1', Int32MultiArray, edit_corners_rob1_callback)
    rospy.Subscriber('corners_rob2', Int32MultiArray, edit_corners_rob2_callback)
    rospy.Subscriber('corners_rob3', Int32MultiArray, edit_corners_rob3_callback)
    rospy.Subscriber('corners_rob4', Int32MultiArray, edit_corners_rob4_callback)

def global_talker():
    # Initialize the node
    rospy.init_node('formation_global_knowledge')
    rospy.loginfo("%s started" % rospy.get_name())

    # Defining Publishers
    alignPub = rospy.Publisher('align_axis', Int32MultiArray, queue_size=10)
    shapeCorners = rospy.Publisher('shape_corner_robots', Int32MultiArray, queue_size=10)

    shapeLength = rospy.Publisher('shape_length', Int32, queue_size=10)
    shapeSides = rospy.Publisher('shape_sides', Int32, queue_size=10)

    xSides = rospy.Publisher('x_sides', Int32MultiArray, queue_size=10)
    ySides = rospy.Publisher('y_sides', Int32MultiArray, queue_size=10)

    while not rospy.is_shutdown():
        # Publish all Data to the Network
        align_axis_mirror = Int32MultiArray()
        shape_corner_mirror = Int32MultiArray()
        x_sides_mirror = Int32MultiArray()
        y_sides_mirror = Int32MultiArray()

        align_axis_mirror.data = np.reshape(align_axis, (8))
        shape_corner_mirror.data = np.reshape(shape_corner_robots, (8))
        x_sides_mirror.data = x_sides
        y_sides_mirror.data = y_sides

        alignPub.publish(align_axis_mirror)
        shapeCorners.publish(shape_corner_mirror)

        shapeLength.publish(shape_length)
        shapeSides.publish(shape_sides)
        xSides.publish(x_sides_mirror)
        ySides.publish(y_sides_mirror)

        # Ensuring that the node subscribes to new data
        global_node_listener()

if __name__ == '__main__':
    try:
        shape_setter()
        global_node_listener()
        global_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
