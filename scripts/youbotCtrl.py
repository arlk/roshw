from __future__ import division
import numpy as np
import math
import rospy
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import simple_world
from planner import potentialPlanner

obs = simple_world.obstacles()
obs.calcVertices()

potField = potentialPlanner()

x = np.zeros(3)
goal = np.array([5,5])
robotDim = [0.25, 0.5]
u = np.zeros(3)


def posnData(data):
    global x
    x[0] = data.pose.pose.position.x
    x[1] = data.pose.pose.position.y

    quat = np.array([0.,0.,0.,1.])
    quat[0] = data.pose.pose.orientation.x
    quat[1] = data.pose.pose.orientation.y
    quat[2] = data.pose.pose.orientation.z
    quat[3] = data.pose.pose.orientation.w

    x[2] = euler_from_quaternion(quat)[2]

def robotCorners(angle):
    rotMatrix = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])

    robot = []
    robot.append([ robotDim[0],-robotDim[1]])
    robot.append([ robotDim[0],robotDim[1]])
    robot.append([ -robotDim[0],robotDim[1]])
    robot.append([ -robotDim[0],-robotDim[1]])
    robot = np.asarray(robot)/2.
    robot = np.dot(rotMatrix,robot.T)

    return robot.T + x[:2]

def rotTransform(pose):
    rotMat = np.zeros((2,2))
    rotMat[:,:] = math.cos(x[2])
    rotMat[0,1] = math.sin(x[2])
    rotMat[1,0] = -math.sin(x[2])
    return rotMat

def controller(rotate):
    global u
    posn = x[:2] 
    # u /= 0.1*np.linalg.norm(u)
    potCmd = np.dot(rotate,u[:2])
    print "SUM JACOBIAN:", u[:2]
    # print potCmd, u, x[2]

    cmd = Twist()
    cmd.linear.x = potCmd[0]
    cmd.linear.y = potCmd[1]
    cmd.angular.z = u[2]
    print "Angle: ",u[2]
    print "Vel", potCmd, u[:2]
    return cmd
    
def sendCmd():
    global u
    while not rospy.is_shutdown():
        rot = rotTransform(x)
        cntrlPts = robotCorners(x[2])
        u = potField.planTraj(cntrlPts, x[2], x[:2], obs.vertices, goal)
        msg = controller(rot)
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