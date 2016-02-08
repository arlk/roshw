#!/usr/bin/env python

__author__ = 'arun lakshmanan'

import numpy as np
import math
import rospy
import time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

def posnData(data):
    global x
    x[0] = data.x
    x[1] = data.y
    x[2] = data.theta
    
def sendCmd():
    while not rospy.is_shutdown():
        rot = rotTransform(x)
        msg = controller(rot)
        pub.publish(msg)
        rate.sleep()

def rotTransform(pose):
    rotMat = np.zeros((2,2))
    rotMat[:,:] = math.cos(x[2])
    rotMat[0,1] = math.sin(x[2])
    rotMat[1,0] = -math.sin(x[2])
    return rotMat

def controller(rotate):
    global gInput, goal
    posn = x[:2] 
    relPosn = np.dot(rotate,goal-posn)
    cmd = Twist()
    if np.linalg.norm(relPosn)<0.2:
        gInput = raw_input('Goal Reached. Enter new goal x, y:')
        goal = np.fromstring(gInput, sep=',')
        relPosn[:]=0
    cmd.linear.x = 0.8*relPosn[0] 
    cmd.angular.z = 0.7*relPosn[1]
    return cmd

if __name__=='__main__':
    rospy.init_node('controller')
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('turtle1/pose', Pose, posnData)
    rate = rospy.Rate(100)
    x = np.zeros(3)
    gInput = raw_input('Enter goal x, y:')
    goal = np.fromstring(gInput, sep=',')
    try:
        sendCmd()
    except rospy.ROSInterruptException:
        pass
