#!/usr/bin/env python

"""
Created, tested and maintained by Rohit for RBE 501 Final Project with JACO arm
"""

import rospy
from numpy import *
from math import *
from sensor_msgs.msg import JointState
from jacoTF import *
from jacoarm.msg import traj_params

"""
The purpose of this node is to create and generate a set of waypoints for the
JACO arm to follow, i.e. to create a discretized trajectory. 
"""

class trajectory():
    
    def axisD(self, p0, pf, v0, vf, a0, af, i):
        axisD = [p0[i], v0[i], a0[i], pf[i], vf[i], af[i]]
        return mat(axisD).T
    
    def pdt(self, t, matrix = False):
        if not matrix:
            return [ 1, t, t**2,   t**3,    t**4,    t**5]
        else:
            return mat([ 1, t, t**2,   t**3,    t**4,    t**5])

        
    def vdt(self, t, matrix = False):
        if not matrix:
            return [ 0, 1, 2*t, 3*(t**2),  4*(t**3),  5*(t**4)]
        else:
            return mat([ 0, 1, 2*t, 3*(t**2),  4*(t**3),  5*(t**4)])
        
    def adt(self, t, matrix = False):
        if not matrix:
            return [ 0, 0,   2,   6*t, 12*(t**2), 20*(t**3)]
        else:
            return mat([ 0, 0,   2,   6*t, 12*(t**2), 20*(t**3)])
        
    def buildA(self, t0, tf):
        A = [self.pdt(t0),
             self.vdt(t0),
             self.adt(t0), 
             self.pdt(tf),
             self.vdt(tf), 
             self.adt(tf)]
        return mat(A)
    
    def getWaypoint(self, t):
        wp = mat([self.pdt(t),
                  self.vdt(t),
                  self.adt(t)])
        return wp
    
    def genTrajectories(self):
        
        xTraj = []
        yTraj = []
        zTraj = []
        for i in self.xfrange(self.t0, self.tf, self.tstep):
             xTraj.append([(self.pdt(i, True) * self.xcoeff).item(0), 
                           (self.vdt(i, True) * self.xcoeff).item(0), 
                           (self.adt(i, True) * self.xcoeff).item(0)])
             yTraj.append([(self.pdt(i, True) * self.ycoeff).item(0), 
                           (self.vdt(i, True) * self.ycoeff).item(0), 
                           (self.adt(i, True) * self.ycoeff).item(0)])
             zTraj.append([(self.pdt(i, True) * self.zcoeff).item(0), 
                           (self.vdt(i, True) * self.zcoeff).item(0), 
                           (self.adt(i, True) * self.zcoeff).item(0)])
        
        
    def xfrange(self, start, stop, step):
        while not rospy.is_shutdown() and start < stop:
            yield start
            start += step
        
    def __init__(self, p0, pf, v0, vf, a0, af, t0 = 0, tf=5,tstep = 0.05):
        # Initialize Node
        rospy.init_node('rbansal_srao_Trajectory')
        
        # Setup publisher and Subscriber
        #self.optmap_pub = rospy.Publisher('/map_Opt', OccupancyGrid, latch=True)
        
        xd = self.axisD(p0, pf, v0, vf, a0, af, 0)
        yd = self.axisD(p0, pf, v0, vf, a0, af, 1)
        zd = self.axisD(p0, pf, v0, vf, a0, af, 2)
        
        A = self.buildA(t0, tf)
        Ainv = linalg.pinv(A)
        self.xcoeff = Ainv * xd
        self.ycoeff = Ainv * yd
        self.zcoeff = Ainv * zd
        
        self.t0 = t0
        self.tf = tf
        self.tstep = tstep
        
# This is the program's main function
if __name__ == '__main__':
    node = trajectory((0,0,0), (1,0,.8), (0,0,0),(0,0,0),(0,0,0),(0,0,0))
    node.genTrajectories()
    rospy.spin()
