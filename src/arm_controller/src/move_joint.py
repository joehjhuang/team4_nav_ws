#!/usr/bin/python

import dynamixel_msgs.msg
import sensor_msgs.msg
import std_msgs.msg
import message_filters
import rospy
import math as m
import numpy as np

class Move_Joint:

    def __init__(self):
        self.states_sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState,self.angles_callback,queue_size=1) 
        #self.target_sub = rospy.Subscriber("/target", Vector3,self.target_callback,queue_size=1) 
        #self.gripping_sub = rospy.Subscriber("/gripping", std_msgs/Bool,self.gripping_callback,queue_size=1) 

        self.joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)
        
        #current position (only care about z position)
        self.position=np.array([0.0,0.0,0.0])
        #target position (only care about z position)
        self.target=np.array([0.0,0.0,0.0])
        #gripper control (false=open, true=close)
        self.gripping=False
        #length from base to center of gripper
        self.l1=0.22
        
                
    def angles_callback(self,data):
        #angles=np.array(data.position)
        #self.position[2]=self.l1*m.cos(angles[0])
        #if self.initial == False:
            #self.set_joint_pos(self.initial_pos[0],self.initial_pos[1])
            #if abs(pos[0]-self.initial_pos[0])<=1e-2 and abs(pos[1]-self.initial_pos[1])<=1e-2:
                #self.initial = True
            #print 'going to initial condition'
            #return
        #print 'going to target'
        self.set_joint_pos(0.3,-2.9)
        
    def set_joint_pos(self,j1,j2):
        self.joint1_pub.publish(std_msgs.msg.Float64(j1))
        self.joint2_pub.publish(std_msgs.msg.Float64(j2))
        
    def target_callback(self,data):
        self.target=np.array(data.x,data.y,data.z)
        return
        
    def gripping_callback(self,data):
        self.gripping=data
        return

    
if __name__ == '__main__':
    rospy.init_node("joint_mover",anonymous=True)
    node = Move_Joint()
    rospy.spin()
    
