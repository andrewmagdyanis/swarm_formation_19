#!/usr/bin/env python
from __future__ import division
import rospy
import math
import time
import numpy as np
from std_msgs.msg import Float32, Int32MultiArray, Float32MultiArray, Int32

"""
  ** RANGE DETECTOR SIMULATOR **
  This node is responsible for simulating a range detector camera
  The node simulates a range detector of angle 180 and a distance m
  in front of the robot

  The node takes robot positions as input and returns a list that contains
  the number of robots each robot can see

  This node will contain the algorithm needed to set a leader
"""

# Global variables for robot positions
rob1_position = [0]
rob2_position = [0]
rob3_position = [0]
rob4_position = [0]

# Robot Range States
rob1_range_state = 0
rob2_range_state = 0
rob3_range_state = 0
rob3_range_state = 0

# Distance to detect infront of robot in cm
m = 87

def rob1_pos_callback():
    global rob1_position
    rob1_position = data.data

def rob2_pos_callback():
    global rob2_position
    rob2_position = data.data

def rob3_pos_callback():
    global rob3_position
    rob3_position = data.data

def rob4_pos_callback():
    global rob4_position
    rob4_position = data.data

def vertices_calc(rob_position):
    # Get the Center of that rectangle
    robot_angle = rob_position[2] / 100         #Because Angle is sent in rad and multiplied by 100
    x_offset = 0.5 * m * ( math.cos( (0.5 * math.pi) - robot_angle ) )
    y_offset = 0.5 * m * ( math.sin( (0.5 * math.pi) - robot_angle ) )
    rect_center = [ rob_position[0] - x_offset , rob_position[1] + y_offset ]

    # Calculating Vertices
    A = []
    A[0] = rect_center[0] - ( m * math.cos(robot_angle) ) - ( 0.5 * m * math.sin(robot_angle) )  #X coordinate
    A[1] = rect_center[1] - ( m * math.sin(robot_angle) ) + ( 0.5 * m * math.cos(robot_angle) )  #Y coordinate

    B = []
    B[0] = rect_center[0] + ( m * math.cos(robot_angle) ) - ( 0.5 * m * math.sin(robot_angle) )  #X coordinate
    B[1] = rect_center[1] + ( m * math.sin(robot_angle) ) + ( 0.5 * m * math.cos(robot_angle) )  #Y coordinate

    C = []
    C[0] = rect_center[0] + ( m * math.cos(robot_angle) ) + ( 0.5 * m * math.sin(robot_angle) )  #X coordinate
    C[1] = rect_center[1] + ( m * math.sin(robot_angle) ) - ( 0.5 * m * math.cos(robot_angle) )  #Y coordinate

    D = []
    D[0] = rect_center[0] - ( m * math.cos(robot_angle) ) + ( 0.5 * m * math.sin(robot_angle) )  #X coordinate
    D[1] = rect_center[1] - ( m * math.sin(robot_angle) ) - ( 0.5 * m * math.cos(robot_angle) )  #Y coordinate

    vertices = [A, B, C, D]
    return vertices

def point_check(rect_vertices, point):
    ## Function checks that a certain point lies within the boundaries of a rectangle
    # Area of the rectangle is a constant since range is constant
    rect_area = m * (2 * m)

    # Calculate area from the point
    Px = point[0]
    Py = point[1]

    Ax = rect_vertices[0][0]
    Ay = rect_vertices[0][1]

    Bx = rect_vertices[1][0]
    By = rect_vertices[1][1]

    Cx = rect_vertices[2][0]
    Cy = rect_vertices[2][1]

    Dx = rect_vertices[3][0]
    Dy = rect_vertices[3][1]

    #A = A   B = P   C = D
    area_apd = abs( (Px * Ay - Ax * Py) + (Dx * Px - Px * Dx) + (Ax * Dy - Dx * Ay) ) / 2

    #A = D  B = P   C = C
    area_dpc = abs( (Px * Dy - Dx * Py) + (Cx * Px - Px * Cx) + (Dx * Cy - Cx * Dy) ) / 2

    #A = C   B = P   C = B
    area_cpb = abs( (Px * By - Bx * Py) + (Bx * Px - Px * Bx) + (Cx * By - Bx * Cy) ) / 2

    #A = P   B = B   C = A
    area_pba = abs( (Bx * Py - Px * By) + (Ax * Bx - Bx * Ax) + (Px * Ay - Ax * Py) ) / 2

    # If calculated areas are within a threshold it's inside
    area_sum = area_apd + area_dpc + area_cpb + area_pba

    if abs ( rect_area - area_sum ) <= 10:
        return 1
    else:
        return 0

def in_range_count(current_robot, other_robots):
    # Check first that the robot position is modified
    # not the standard NULL form
    if len(current_robot == 0):
        return 0
    else:
        # Calculate Bounding Rectangle
        bounding_vert = vertices_calc(current_robot)

        # Loop over robot positions and check if they are inside
        for i in range(1, 4):
            counter += point_check(bounding_vert, other_robots[i])

        return counter

def range_count_calculator():
    global rob1_range_state, rob2_range_state, rob3_range_state, rob4_range_state
    #Save current robot positions to avoid errors from fluctuations
    rob1_pos = rob1_position
    rob2_pos = rob2_position
    rob3_pos = rob3_position
    rob4_pos = rob4_position

    #Calculating for Robot 1
    other_robots1 = [rob2_pos, rob3_pos, rob4_pos]
    rob1_range_state = in_range_count(rob1_pos, other_robots)

    #Calculating for Robot 2
    other_robots2 = [rob1_pos, rob3_pos, rob4_pos]
    rob2_range_state = in_range_count(rob2_pos, other_robots)

    #Calculating for Robot 3
    other_robots3 = [rob1_pos, rob2_pos, rob4_pos]
    rob3_range_state = in_range_count(rob3_pos, other_robots)

    #Calculating for Robot 4
    other_robots4 = [rob1_pos, rob2_pos, rob3_pos]
    rob4_range_state = in_range_count(rob4_pos, other_robots)

def range_listener():
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, rob1_pos_callback)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, rob2_pos_callback)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, rob3_pos_callback)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, rob4_pos_callback)

def range_talker():
    rospy.init_node('range_detector_sim')
    rospy.loginfo("%s started" % rospy.get_name())

    # Run the subscribers first time
    range_listener()

    rob1 = rospy.Publisher('rob1_range_state', Int32, queue_size=10)
    rob2 = rospy.Publisher('rob2_range_state', Int32, queue_size=10)
    rob3 = rospy.Publisher('rob3_range_state', Int32, queue_size=10)
    rob4 = rospy.Publisher('rob4_range_state', Int32, queue_size=10)

    while not rospy.is_shutdown():
        # Make Calculations of Ranges for Each robot
        range_count_calculator()

        # Publish the Current States
        rob1.publish(rob1_range_state)
        rob2.publish(rob2_range_state)
        rob3.publish(rob3_range_state)
        rob4.publish(rob4_range_state)

        # Ensure subscribers are run
        range_listener()

if __name__ == '__main__':
    try:
        range_talker()
    except ros.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed." % rospy.get_name())
