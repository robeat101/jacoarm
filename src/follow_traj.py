#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_driver')
import rospy
import actionlib
import jaco_msgs.msg
import sys

class follow_traj():
        
    def traj_callback(self, msg):
        px_traj = msg.posx_traj 
        py_traj = msg.posy_traj 
        pz_traj = msg.posz_traj 
        
        if(len(px_traj) != len(py_traj) or len(px_traj) != len(zx_traj)):
            rospy.logwarn("The lengths of the trajectories don't match. Aborting")
            return
        else:
            for i in xrange(0, len(px_traj)):
                if(rospy.is_shutdown())
                    rospy.logwarn("Shutdown request received")
                    return 
                self.pose_client(px_traj[i], py_traj[i], pz_traj[i])
                
        
    def pose_client(self, x, y, z):
        
        goal.pose.header.frame_id = "/jaco_api_origin"
        
        goal.pose.pose.orientation.x = -0.590686044496
        goal.pose.pose.orientation.y = -0.519369415388
        goal.pose.pose.orientation.z = 0.324703360925
        goal.pose.pose.orientation.w = 0.525274342226
        
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.position.z = z

        client.wait_for_server()
        rospy.loginfo("Connected to Pose server")
    
        client.send_goal(goal)
    
        return
    
    def __init__(self):

        # Initialize Node
        rospy.init_node('rbansal_srao_follow_traj')
        
        self.px_traj = []
        self.py_traj = []
        self.pz_traj = []
        
        self.client = actionlib.SimpleActionClient('/jaco/arm_pose', jaco_msgs.msg.ArmPoseAction)
        self.goal = jaco_msgs.msg.ArmPoseGoal()
        
        sub = rospy.Subscriber('/rbe_jacoapi/trajectories', trajectorymsg, self.traj_callback ,queue_size=1)
        

if __name__ == '__main__':
    try:
        rospy.init_node('arm_pose_client')
        result = pose_client()
        rospy.loginfo("Result: %s", result)
    except rospy.ROSInterruptException: 
        rospy.loginfo("Program interrupted before completion")
        
