#!/usr/bin/python
"""
2.12 Navigation: The robot follows a line of given waypoints to reach the target.

A state machine is used for the navigation task:

compute distance to target
    if d<=epsilon -> done
    else          -> check if on line
check if on line
    if yes -> check if heading is correct
    if no  -> adjust theta to theta_correction
check if heading is correct
    if yes -> CRANK IT
    if no  - > adjust theta to theta_heading

#functional blocks
#   distance_to_target
#       d=sqrt((xt-x)^2+(yt-y)^2)
#   distance_to_line
#       abs(a*x+b*y+c)/sqrt(a^2+b^2)
#   heading_to_target
#       if abs(theta_robot-theta_setpoint) < theta_threshold
#           on target
#       else
#           off target
#   adjust_theta(setpoint)   
#   set_speed
#   compute_line(start_coords, end_coords)
#       write line as ax+by+c=0
#       m=(y1-y0)/(x1-x0)
#       a=m
#       b=-1
#       c=y0-x0*m
"""

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from robot_base_controller.msg import WheelCmdVel
import math as m

class Navigate:
    """
    The node that navigates the robot to the target via a set of waypoints.
        Subscribe topics:
            /estimate_position/pose: geometry_msgs/Vector3
            /estimate_position/angle: geometry_msgs/Vector3
        Publish topics:
            /cmdvel: WheelCmdVel
    """
    def __init__(self):
        self.pose_sub=rospy.Subscriber("/estimate_position/pose",Vector3,self.pose_callback,queue_size=1)
        self.angle_sub=rospy.Subscriber("/estimate_position/angle",Vector3,self.angle_callback,queue_size=1)
        self.cmd_pub=rospy.Publisher("/cmdvel",WheelCmdVel,queue_size=1)
        
        self.waypoint_sub = rospy.Subscriber("/current_waypoint",Vector3,self.waypoint_callback,queue_size=1)
        
        self.task = 0
        self.counter_pub = rospy.Publisher("/nav_counter",Vector3,queue_size=1)
        self.current_waypoint = np.array([0,0])
        self.trajectory = self.current_waypoint

        self.pose_ready = False #pose data available?
        self.angle_ready = False #angle data available?  
        self.current_position=np.array([0.,0.,0.])

        
        self.counter=0 #iterates over all waypoints
        self.current_target = self.trajectory #next waypoint
        self.arrived = True
        
        self.angle_gain=0.1
        self.distance_gain=0.005
        self.theta_bearing=0
        self.distance=0
        
        self.pub_wheelvel(0,0)
        self.b = 0.225 #Wheel-to-centerline distance (m)
        self.r = 0.037 #Wheel radius (m)
    	self.epsilon=0.10 #max distance to target

    def pose_callback(self,data):
        """
        Records data from position estimator
            Input:
                data: position data from estimator
            Output:
                no output
        """
        self.pose_ready=True
        self.current_pose=np.array([data.x,data.y,data.z])
        return
        
    def task_callback(self, data):
        self.task = data
        return

    def angle_callback(self,data):
        """
        Records data form angle estimator (main node that runs navigation state machine)
            Input:  
                data: angle from estimator
            Output:
                no output 
        """
        print "in angle callback function!"
        self.angle_ready=True
        self.current_angle=np.array([data.x,data.y,data.z])
        
        if self.pose_ready and self.angle_ready and not self.arrived:
            self.current_position=self.current_pose+self.current_angle #current position array (x,y,theta)
            
            #start the state machine
            self.distance=self.dist_to_target()
            #have reached target
            if self.distance<self.epsilon:
                self.counter=self.counter+1
                self.counter_pub.publish(np.array([self.counter,0,0]))
                #reached the end of trajectory
                self.pub_wheelvel(0.,0.)
                self.arrived = True
                #
                #self.waypoint_update() <- probably unnecessary
                #not reached end of trajectory yet
            else:
                self.waypoint_update()
            #not reached target yet
        else:
            #compute new bearing
            gamma=m.atan2(self.current_target[1]-self.current_position[1],self.current_target[0]-self.current_position[0])
            print " robot position", self.current_position
            print "target position", self.current_target
            self.theta_bearing = gamma - self.current_position[2]
            print "theta_bearing: ", self.theta_bearing
            print "angle of robot: ", self.current_position[2]
            print "angle of target: ", gamma
            self.set_wheelvel()
        return 

    def waypoint_update(self):
        """
        update the start and target point for new segment
        Input:
            self
        Output:
            no output
        """
        self.current_target=self.current_waypoint
        self.trajectory = self.current_waypoint #contains all waypoints as (x,y)
        self.current_target = self.trajectory
        return
        
    def waypoint_callback(self,data):
        print('nav_state received a waypoint!')
        self.current_waypoint = np.array([data.x,data.y])
        self.arrived = False
        return
 
    def dist_to_target(self):
        """
        Computes the distance from the current position to the target
            Input:
                self: require target and robot position
            Output:
                distance to target
        """
        x_r=self.current_position[0]
        y_r=self.current_position[1] 

        x_t=self.current_target[0]
        y_t=self.current_target[1] 
             
        d=m.sqrt((x_r-x_t)**2+(y_r-y_t)**2)
        return d

    def pub_wheelvel(self,wr,wl):
        wheelcmd = WheelCmdVel()
        wheelcmd.desiredWV_R = wr
        wheelcmd.desiredWV_L = wl
        self.cmd_pub.publish(wheelcmd)
        print('wheel velocities:',wr,', ', wl)
        return

    def set_wheelvel(self):
        phi_dot=self.angle_gain*self.theta_bearing
        print "robot turn rate", phi_dot
        fwdvel=self.distance_gain*self.distance
        w_r=fwdvel/self.r+self.b/self.r*phi_dot
        w_l=fwdvel/self.r-self.b/self.r*phi_dot
        
        self.pub_wheelvel(w_r,w_l)

if __name__ == '__main__':
    rospy.init_node("navigator",anonymous=True)
    node = Navigate()
    rospy.spin()
