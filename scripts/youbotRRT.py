from __future__ import division
import numpy as np
import math
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
time = 0


def posnData(data):
    global x
    x[0] = data.pose.pose.position.x
    x[1] = data.pose.pose.position.y


def controller(time):
    global u
    int Kv = 0.1
    currentPos = x
    desiredPos = ptList[time]

    cmdVel = Kv * (desiredPos - currentPos)

    cmd = Twist()
    cmd.linear.x = cmdVel[0]
    cmd.linear.y = cmdVel[1]
    print "Vel", potCmd, u[:1]
    return cmd

def sendCmd():
    global u,time
    while not rospy.is_shutdown():
        time += 1                            #TIME TIME TIME
        msg = controller(time)
        pub.publish(msg)
        rate.sleep()

if __name__=='__main__':
    rospy.init_node('controller')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/odom', Odometry, posnData)
    rate = rospy.Rate(100)

    try:
        sendCmd()
    except rospy.ROSInterruptException:
        pass
