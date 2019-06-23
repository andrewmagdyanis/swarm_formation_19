#!/usr/bin/env python

import argparse
import sys
import rospy
from std_msgs.msg import Int32MultiArray Int32 String Float32
from rospy_tutorials.msg import Floats
import numpy
import math
import os
###############################################################################
#initialize all variables:
###############################################################################
#robots numbers:
leader_num=0
follower1_num=1
follower1_num=2
follower1_num=3
#desired orientation difference:
Ad2 = 0
Ad3 = 0
#desired distance:
Dd=0
c2c_distance_px=1 # it is published from the user input and if not it by default =1
grid_dim=17.5 #cm
grid_dig_dim=24.75 #cm
swarm_robots_num=4 # this year we have 4 robots
#orientation in radians:
A2=0
A3=0
#formation shape:
shapes = ''

#leader new position after check in cm:
R1_gx_cm = 0
R1_gy_cm =0
#leader final position
rob1_goal_x =0
rob1_goal_y =0
#leader current position:
x1=0
y1=0
a1=0
#robot 2 current position:
x2=0
y2=0
#robot 3 current position:
x3=0
y3=0
#robot 4 current position:
x4=0
y4=0
#incrementation values:
m=0
n=0
###############################################################################
#RobotClass:
###############################################################################
class RobotID:
    name='robot '
    ID= -1
    IP='192.168.1. '
    status=''
    current_goal= [[0,0],[0,0],[0,0],[0,0]]
    next_goal= [[0,0],[0,0],[0,0],[0,0]]
    def __init__(self, name, ID,status):
        self.name = name
        self.ID = ID
        self.status=status

    def set_name(self, new_name):
        self.name = new_name
    def set_ID(self, ID):
        self.ID = ID
    def set_robot_IP(self, IP):
        self.IP = IP
    def set_status(self, status):
        self.status =status
    def set_current_goal(self, cgoal):
        self.current_goal[self.ID] = cgoal
    def set_next_goal(self, ngoal):
        self.next_goal[self.ID] = ngoal

    def get_name(self):
        return self.name
    def get_ID(self):
        return self.ID
    def get_IP(self):
        return self.IP
    def get_status(self):
        return self.status
    def get_next_goal(self):
        return self.next_goal[self.ID]
    def get_current_goal(self):
        return self.current_goal[self.ID]

###############################################################################
#define publishers:
###############################################################################
pub  = rospy.Publisher('robot1_goal_px', Int32MultiArray, queue_size=10)

pub1 = rospy.Publisher('robot2_goal_px', Int32MultiArray, queue_size=10)
pub2 = rospy.Publisher('robot3_goal_px', Int32MultiArray, queue_size=10)
pub3 = rospy.Publisher('robot4_goal_px', Int32MultiArray, queue_size=10)

pub4 = rospy.Publisher('robot2_goal_cm', Int32MultiArray, queue_size=10)
pub5 = rospy.Publisher('robot3_goal_cm', Int32MultiArray, queue_size=10)
pub6 = rospy.Publisher('robot4_goal_cm', Int32MultiArray, queue_size=10)

pub7 = rospy.Publisher('m_n_values', Int32MultiArray, queue_size=10)
###############################################################################
#callback functions:
###############################################################################
def callback1(data): # formation shape:
	global shapes
	shapes = data.data

def callback2(data): #center to center distance in pixels
	global c2c_distance_px
	c2c_distance_px = data.data

def callback3(data): #current position of robot1 (leader)
	 global x1, y1, a1
	 #rospy.loginfo('robot1 co. = %s', data.data)
	 x1 = data.data[0]
	 y1 = data.data[1]
	 a1 = data.data[2]

def callback4(data): #current position of robot 2
	 global x2, y2
	 #rospy.loginfo('robot2 co. = %s', data.data)
	 x2 = data.data[0]
	 y2 = data.data[1]

def callback5(data): #current position of robot 3
	global x3, y3
	#rospy.loginfo('robot3 co. = %s', data.data)
	x3 = data.data[0]
	y3 = data.data[1]

def callback6(data): #current position of robot 4
	global x4, y4
	#rospy.loginfo('robot4 co. = %s', data.data)
	x4 = data.data[0]
	y4 = data.data[1]

	shape()
	final()

def callback7(data): # new position of the leader
	global R1_gx_cm, R1_gy_cm
	#rospy.loginfo('robot3 co. = %s', data.data)
	R1_gx_cm = data.data[0]
	R1_gy_cm = data.data[1]

###############################################################################
#define subscribers:
###############################################################################
def listener():
    rospy.init_node('Form')
    rospy.Subscriber('Required_Shape', String, callback1)
    rospy.Subscriber('C2C_distance',Int32,callback2)
    rospy.Subscriber('rob1_CurrentPose', Int32MultiArray, callback3)
    rospy.Subscriber('rob2_CurrentPose', Int32MultiArray, callback4)
    rospy.Subscriber('rob3_CurrentPose', Int32MultiArray, callback5)
    rospy.Subscriber('rob4_CurrentPose', Int32MultiArray, callback6)
    rospy.Subscriber('rob1_final_cm', Int32MultiArray, callback7)

###############################################################################
#shape function:
###############################################################################
def shape():
    ''' this function checks what formation is sent and set
        the orientations and distance between the robots desired.
        also set the incrementations the leader should move if obstacles found
    '''
    global Ad2, Ad3, Ad4, Dd, shapes, m, n #line,column,diagonal
    if shapes == "line" :
        Ad2=0
        Ad3=180
        Ad4=0
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=3
        n=0
    elif shapes == "column" :
        Ad2=90
        Ad3=270
        Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=0
        n=3
    elif shapes == "diagonal" :
        Ad2=45
        Ad3=-135
        Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=3
        n=0
    elif shapes == "triangle":
        Ad2=-45
        Ad3=-135
        Ad4=270
        Dd =swarm_robots_num*c2c_distance_px*grid_dig_dim
        m=2
        n=0
    elif shapes == "L-shape":
        Ad2=0
        Ad3=90
        Ad4=90
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    elif shapes == "square":
        Ad2=0
        Ad3=90
        Ad4=45
        Dd =swarm_robots_num*c2c_distance_px*grid_dim
        m=2
        n=2
    else:
        print "shape is not defined"

    #publish the incrementation values of leader
    m_n_values = numpy.array([ m ,n],Int32MultiArray)
    m_n_values_=Int32MultiArray(data=m_n_values)
    pub7.publish(m_n_values_)

###############################################################################
#calculation function:
###############################################################################
def calculations():

    global A2, A3 ,f13_CM ,f12_CM ,f13_cm ,f12_cm ,f12_px ,f13_px
    #change the angels into radians
    A2 = Ad2*(math.pi/180)
    A3 = Ad3*(math.pi/180)

    #calculate positions of robots 2 and 3 wrt robot 1 position
    x2f = int(round(rob1_goal_x + (Dd*math.cos(A2))))
    x3f = int(round(rob1_goal_x + (Dd*math.cos(A3))))

    y2f = int(round(rob1_goal_y - (Dd*math.sin(A2))))
    y3f = int(round(rob1_goal_y - (Dd*math.sin(A3))))

    #print x2f ,y2f

    #calculate the distances between the current position of the robot and both formations positions
    dA1 = math.sqrt(pow((x2-x2f),2)+pow((y2-y2f),2))
    dA2 = math.sqrt(pow((x2-x3f),2)+pow((y2-y3f),2))
    dB1 = math.sqrt(pow((x3-x2f),2)+pow((y3-y2f),2))
    dB2 = math.sqrt(pow((x3-x3f),2)+pow((y3-y3f),2))
    #set values of the distances in an array
    DA = [dA1 , dA2]
    DB = [dB1 , dB2]

    #sort the distances to find the longest and shortest distance for each robot
    list.sort(DA)
    list.sort(DB)

    #change the final positions into pixels
    x2f_px = math.ceil(((x2f * 2.43308) /42.9))-1
    y2f_px = math.ceil(((y2f * 2.43308) /40.4))-1

    x3f_px = math.ceil(((x3f * 2.43308) /42.9))-1
    y3f_px = math.ceil(((y3f * 2.43308) /40.4))-1

    #check for the longest distance any robot would move
    if DA[1] < DB[1]:
        #send the robot the closest position to it
        if (dB2 > dB1):
            f13_cm = numpy.array([ x2f , y2f , a1 ],Int32MultiArray)
            f12_cm = numpy.array([ x3f , y3f , a1 ],Int32MultiArray)
            f13_px = numpy.array([ x2f_px , y2f_px , a1 ],Int32MultiArray)
            f12_px = numpy.array([ x3f_px , y3f_px , a1 ],Int32MultiArray)
        else:
            f12_cm = numpy.array([ x2f , y2f , a1 ],Int32MultiArray)
            f13_cm = numpy.array([ x3f , y3f , a1 ],Int32MultiArray)
            f12_px = numpy.array([ x2f_px , y2f_px , a1 ],Int32MultiArray)
            f13_px = numpy.array([ x3f_px , y3f_px , a1 ],Int32MultiArray)

    else:
        if (dA2 > dA1):
            f12_cm = numpy.array([ x2f , y2f , a1 ],Int32MultiArray)
            f13_cm = numpy.array([ x3f , y3f , a1 ],Int32MultiArray)
            f12_px = numpy.array([ x2f_px , y2f_px , a1 ],Int32MultiArray)
            f13_px = numpy.array([ x3f_px , y3f_px , a1 ],Int32MultiArray)
        else:
            f13_cm = numpy.array([ x2f , y2f , a1 ],Int32MultiArray)
            f12_cm = numpy.array([ x3f , y3f , a1 ],Int32MultiArray)
            f13_px = numpy.array([ x2f_px , y2f_px , a1 ],Int32MultiArray)
            f12_px = numpy.array([ x3f_px , y3f_px , a1 ],Int32MultiArray)

    #publish the formation positions

    f13_=Int32MultiArray(data=f13_px)
    f12_=Int32MultiArray(data=f12_px)


    f13_CM=Int32MultiArray(data=f13_cm)
    f12_CM=Int32MultiArray(data=f12_cm)

    #		print 'robot2 grid goal' ,f12_px
    #		print 'robot3 grid goal' ,f13_px
    pub1.publish(f12_)
    pub2.publish(f13_)

###############################################################################
#final function:
###############################################################################
def final():
	#print 'cm' ,R1_gx_cm , R1_gy_cm
	if (shapes != '' ):
		global x1 ,y1,rob1_goal_x ,rob1_goal_y

	#if leader has a new position set the position to be added into the calculations as the new position
		if ((R1_gx_cm > 0) and (R1_gy_cm > 0)):

			x1= R1_gx_cm
			y1= R1_gy_cm
			rob1_goal_x = x1
			rob1_goal_y = y1
	#call the calculation function
			calculations()
			print 'robot2 goal' ,f12_cm
			print 'robot3 goal' ,f13_cm
	#publish the final formation positions
			pub4.publish(f12_CM)
			pub5.publish(f13_CM)
		else:
	#if leader isnt supposed to move set the positions to be added into the calculations as current position of the leader
			rob1_goal_x = x1
			rob1_goal_y = y1
			calculations()
	else:

		print 'waiting required_shape'


#change the positions into pixels and publish it

	rob1_goal_x_px = math.ceil(((rob1_goal_x * 2.43308) /42.9))-1
	rob1_goal_y_px = math.ceil(((rob1_goal_y * 2.43308) /40.4))-1
	rob1_goal = numpy.array([rob1_goal_x_px ,rob1_goal_y_px ],Int32MultiArray)
	R1_goal=Int32MultiArray(data=rob1_goal)
	#print rob1_goal
	pub.publish(R1_goal)

###############################################################################
#justify_distance function:
###############################################################################
def justify_distance(next,base,length):
    ''' (next) point is used as the follower point,
        (base) is used as leader point,
        (length) is used as shape length
        this function justify the goal position of the robot to the shape length
        by checking if the desired position above/below shape length
        then the goal is updated accordingly to a point that
        meet the shape length requirement.
    '''
    #1. initialize justified Point to 0,0:
    justified_point[0]=0 # x of the point
    justified_point[1]=0 # y of the point
    #2. check that next and bas sharing the same x:
    if next[0]==base[0]:
        justified_point[0]= base[0]
        if base[1]<next[1]: #next is above the base
            justified_point[1]=base[1]+length
        else: #next is below
            justified_point[1]=base[1]-length
    #3. check that next and bas sharing the same y:
    elif next[1]==base[1]:
        justified_point[1]= base[1]
        if base[0]<next[0]: #next is above the base
            justified_point[0]=base[0]+length
        else:
            justified_point[0]=base[0]-length
    else:
        print "no justify yet(error)"
    return justified_point

###############################################################################
#align function:
###############################################################################
def align(follower,follower_num,leader,direction):
    ''' this function determine the goal position for follower based on alignment direction.
        The robot align in 90 degree in x or y direction. The
        function takes direction, follower and leader position anddetermine the next goal point according to direction
        specified. The resulted goal position is then modified
        according to shape length by calling a justify_distance
        function. align_axis global flags are updated after goal
        position is chosen for both follower and leader
    '''
    #align in x or y as demand:
    if (direction=='x'):
        #1. align the next_goal x index with leader x index:
        next_goal[follower_num][0]=leader[0]
        #2. leave the y index of the next_goal as the follower:
        next_goal[follower_num][1]=follower[1]
        #3. set the align axis:
        align_axis[leader_num][0]=1
        align_axis[follower_num][0]=1
    elif (direction=='y'):
        #1. align the next_goal y index with leader y index:
        next_goal[follower_num][1]=leader[1]
        #2. leave the x index of the next_goal as the follower:
        next_goal[follower_num][0]=follower[0]
        #3. set the align axis:
        align_axis[leader_num][1]=1
        align_axis[follower_num][1]=1
    #update next_goal of the follower:
    next_goal[follower_num]=justify_distance(next_goal[follower_num],leader,shape_length)

###############################################################################
#Main:
###############################################################################
if __name__== '__main__':
	while not rospy.is_shutdown():
		listener()
	#	rate.sleep()
		rospy.spin()
