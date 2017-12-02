#!/usr/bin/python

import rospy
import numpy as np
import std_msgs
from geometry_msgs.msg import Vector3
from robot_base_controller.msg import WheelCmdVel
import math as m

class bottle_grab:

    def __init__(self):
        self.angle_sub=rospy.Subscriber("/estimate_position/angle",Vector3,self.angle_callback,queue_size=1)
        self.bottle_sub=rospy.Subscriber("/object_location/base",Vector3,self.bottle_callback,queue_size=1)
        
        self.cmd_pub=rospy.Publisher("/cmdvel",WheelCmdVel,queue_size=1)
        self.joint1_pub = rospy.Publisher('/joint1_controller/command', std_msgs.msg.Float64, queue_size=1)
        self.joint2_pub = rospy.Publisher('/joint2_controller/command', std_msgs.msg.Float64, queue_size=1)
        
        self.bottle_location=np.array([0.0,0.0,0.0])
        self.dist_tol = 0.05
        self.counter = 0.0
        self.average = 5.0
        self.gain = 0.0001
        self.y_target=320
        self.dist_target = 0.25
        
        self.fwdvel = 0.0016
    
        self.initial_angle=False
        self.theta_tol = m.pi/6.
        self.theta = 0.0
        self.theta_initial = m.pi
        self.turnrate = 0.5
        self.b=0.225
        self.r = 0.037
        
        self.joint1=2.75
        self.joint2=-2.9
        
    def angle_callback(self,data):
        self.theta=data.z
        print "initial angle: ", self.initial_angle
        if self.initial_angle:
            self.set_joint_pos(self.joint1,self.joint2)
            return
        else:
            if abs(self.theta-self.theta_initial) <= self.theta_tol:
                self.initial_angle=True
                self.set_joint_pos(self.joint1,self.joint2)
                self.pub_wheelvel(0.0,0.0)
            elif self.theta < self.theta_initial:
                wr=self.b*self.turnrate
                wl=-wr
                self.pub_wheelvel(wr,wl)
                print "adjusting angle left"
            elif self.theta > self.theta_initial:
                wl=self.b*self.turnrate
                wr=-wl
                self.pub_wheelvel(wr,wl)
                print "adjusting angle right"
    
    def bottle_callback(self,data):
        self.counter=self.counter+1
        #print "counter: ", self.counter
        #bottle location wrt. the base frame [distance [m], horizontal [pixel], vertical [pixel]]
        self.bottle_location=self.bottle_location+np.array([data.x,data.y,data.z])
        #print "bottle_location: ", self.bottle_location
        if self.counter == self.average:
            self.bottle_location=1.0/self.average*self.bottle_location
            self.counter=0
            #print "bottle_location: ", self.bottle_location
            if self.initial_angle==True:
                self.adjust_position()
                print "distance ", self.bottle_location[0]
            self.bottle_location=np.array([0.0,0.0,0.0])
        return
            
    def adjust_position(self):
        if self.bottle_location[0]==0.0:
            self.pub_wheelvel(0.0,0.0)
        elif self.bottle_location[0]<=self.dist_target:
            self.pub_wheelvel(0.0,0.0)
            self.joint2=-2
            self.set_joint_pos(self.joint1,self.joint2)
            print "target reached"

        else:
            phi_dot=self.gain*(self.y_target-self.bottle_location[1])
            print "phi_dot: ", phi_dot
            wr=self.b*(self.fwdvel/self.r+phi_dot)
            wl=self.b*(self.fwdvel/self.r-phi_dot)
            print wr, wl
            self.pub_wheelvel(wr,wl)

    def set_joint_pos(self,j1,j2):
        self.joint1_pub.publish(std_msgs.msg.Float64(j1))
        self.joint2_pub.publish(std_msgs.msg.Float64(j2))

    def pub_wheelvel(self,wr,wl):
        wheelcmd = WheelCmdVel()
        wheelcmd.desiredWV_R = wr
        wheelcmd.desiredWV_L = wl
        self.cmd_pub.publish(wheelcmd)
        return

if __name__== '__main__':
    rospy.init_node("bottle_grabber",anonymous=True)
    node = bottle_grab()
    rospy.spin()
