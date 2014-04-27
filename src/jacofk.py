#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 501 Final Project with JACO arm
"""

import rospy
import copy
from numpy import *
from numpy.linalg import norm
from numpy.linalg import pinv
from math import *
from sensor_msgs.msg import JointState
import tf


#import jacobian as J

"""
The purpose of this node is to read current joint angles from the jaco arm
and publish the cartesian coordinates of the end effector of the JACO arm. 
"""


class jacofk:
    
    D1 = 0.2755
    D2 = 0.4100
    D3 = 0.2073
    D4 = 0.0743
    D5 = 0.0743
    D6 = 0.1687
    e2 = 0.0098
    
    aa = (11.0*pi)/72.0
    ca = cos(aa)
    sa = sin(aa)
    c2a = cos(2*aa)
    s2a = sin(2*aa)
    d4b = (D3 + sa/s2a *D4)
    d5b = (sa/s2a*D4 + sa/s2a *D5)
    d5b = (sa/s2a*D4 + sa/s2a *D5)
    d6b =(sa/s2a*D5 + D6)
    
    alpha = [pi/2, pi, pi/2, 2 * aa, 2 * aa, pi]
    a = [0, D2, 0, 0, 0, 0]
    d = [D1, 0, -e2, -d4b, -d5b, -d6b]
    
    def calcFK(self, msg):
        joint_val = []
        for i in xrange(0,6):
            joint_val.append(msg.position[i])
        q = mat(joint_val)
        q = self.jacotoDH(q)
        T = list()
        T_final = mat(identity(4))
        for i in xrange(0,6):
            T.append(self.transform(q.item(i),
                           jacofk.a[i],
                           jacofk.alpha[i],
                           jacofk.d[i]))
        
        T_final = self.TBtoJB * self.TJBtoJ1 * T[0]*T[1] *T[2] *T[3] *T[4] *T[5] 
        print T_final
        return T_final
        
    def jacotoDH(self, q):
        q[0,0] = -q.item(0)
        q[0,1] = q.item(1) + radians(90)
        q[0,2] = q.item(2) - radians(90)
        q[0,3] = q.item(3)
        q[0,4] = q.item(4) + radians(180)
        q[0,5] = q.item(5) - radians(180 - 80)
        return q
        
        
    def transform(self, theta, a, alpha, d):
        T = [[cos(theta),
              -sin(theta)* cos(alpha),
               sin(theta)* sin(alpha),
               a * cos(theta)]]
        T.append([ sin(theta),
                  cos(theta) * cos(alpha),
                  -cos(theta)* sin(alpha),
                  a * sin(theta)])
        T.append([0,
                  sin(alpha),
                  cos(alpha),
                  d])
        T.append([0,0,0,1])
        T = mat(T)
        return T
        
    def __init__(self):
        # Initialize Node
        rospy.init_node('rbansal_srao_jacofk')
        
        self.TBtoJB = self.transform(pi/2, 0, 0, 0) 
        self.TJBtoJ1= self.transform(0, 0, 0, 0.028)
        # Setup publisher and Subscriber
        #self.endeffectorpose = rospy.Publisher('/map_OE', OccupancyGrid, latch=True)
        self.jointconfig = rospy.Subscriber('/jaco/joint_state', JointState , self.calcFK, queue_size=1)
        
        
# This is the program's main function
if __name__ == '__main__':
    
    node = jacofk()
    rospy.spin()