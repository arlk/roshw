#!/usr/bin/env python
from __future__ import division
import numpy as np
import math
import rospy
import time
from rrtPlanner import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

time = 0
init_posn = np.array([0,0])
x = init_posn
goal_posn = np.array([5,5])
myPlanner = RRT(init_posn,0.05,1)
myPlanner.generate_tree(goal_posn, 0.15)

def posnData(data):
    global x
    x[0] = data.pose.pose.position.x
    x[1] = data.pose.pose.position.y


def controller(time):
    # Kv = 1
    # currentPos = x
    # if time < myPlanner.path.shape[0]:
    #     desiredPos = myPlanner.path[time]
    # else:
    #     desiredPos = myPlanner.path[-1]
    #
    # cmdVel = Kv * (desiredPos - currentPos)
    #
    # cmd = Twist()
    # cmd.linear.x = cmdVel[0]
    # cmd.linear.y = cmdVel[1]
    Kv = 10.0
    cmd = Twist()
    cmd.linear.x = Kv*myPlanner.cntrl[time, 0]
    cmd.linear.y = Kv*myPlanner.cntrl[time, 1]
    return cmd

def sendCmd():
    global time
    while not rospy.is_shutdown():
        # if np.linalg.norm(myPlanner.path[time] - x) <= 0.1:
        #     time += 1                            #TIME TIME TIME
        if time < myPlanner.path.shape[0]-2:
            msg = controller(time)
        else:
            msg = Twist()
        time += 1
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    rospy.init_node('controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/odom', Odometry, posnData)
    rate = rospy.Rate(10)

    try:
        sendCmd()
    except rospy.ROSInterruptException:
        pass
