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
#	    self.task_cmd=rospy.subscriber("/task_cmd",int64,self.task_callback,queue_size=1)

        self.cmd_pub=rospy.Publisher("/cmdvel",WheelCmdVel,queue_size=1)

        self.pub_wheelvel(0,0)

        self.task = 0

        self.pose_ready = False #pose data available?
        self.angle_ready = False #angle data available?  
        self.current_pose=np.array([0.,0.,0.])
        self.current_angle=np.array([0.,0.,0.])

        self.trajectory = np.array([[1.85,1.95],[1.0,1.0]]) #contains all waypoints as (x,y)
        self.counter=1 #iterates over all waypoints
        self.current_start=self.trajectory[0] #first startpoint
        self.current_target=self.trajectory[1] #next waypoint
        self.current_position=np.array([0.,0.,0.])
        #self.last_position=np.array([0.,0.,0.])
    	self.theta_t = 0.;          #Heading of current trajectory line
        self.theta_delta = m.pi/6.  #Correction heading for returning to the line
        self.theta_tol   = m.pi/36. #Tolerance for heading error
        self.line = self.compute_line()
        
        self.b = 0.225 #Wheel-to-centerline distance (m)
        self.r = 0.037 #Wheel radius (m)
        self.fwdvel = .0016 #Forward velocity (m/s)
        self.turnrate = 0.005 #Turning rate (degree/s)
    	self.epsilon=0.05 #max distance to target
        self.delta=0.05 #max distance to line

    def task_callback(self, data):
        self.task = data

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

    def angle_callback(self,data):
        """
        Records data form angle estimator (main node that runs navigation state machine)
            Input:  
                data: angle from estimator
            Output:
                no output 
        """
        print('in angle callback function!')
        self.angle_ready=True
        self.current_angle=np.array([data.x,data.y,data.z])

        if self.pose_ready and self.angle_ready:
            #self.last_position=self.current_position
            self.current_position=self.current_pose+self.current_angle #current position array (x,y,theta)
            
#            if (abs(self.last_position[0]-self.current_position[0])<=1e-6):
#		print('No change detected in position')
#                self.pub_wheelvel(0,0)
#	    print('returning')
#            return
           
            print("Distance to target:",self.dist_to_target())
            print("Distance to line:",self.dist_to_line(self.line))
	    #print(self.trajectory[self.counter])
            #start the state machine
            if self.dist_to_target()<self.epsilon:
                self.counter=self.counter+1
                self.pub_wheelvel(0.,0.)
                if self.counter >= self.trajectory.shape[0]: #reached the end of trajectory
                    self.pub_wheelvel(0.,0.)
                    print "End of Trajectory"
                    return
                else:
                    self.waypoint_update()
                    self.line = self.compute_line()
            else:
                #if self.dist_to_line(self.line) < self.delta:
                    #print "on line"
                self.set_wheelvel(self.fwdvel, self.theta_t)
                #else:
                    #theta_bearing = m.atan2(self.current_target[1]-self.current_position[1],self.current_target[0]-self.current_position[0])
                    
                    #if theta_bearing > self.theta_t:
                        #print "to the right"
                        #theta_r = self.theta_t + self.theta_delta
                    #else:
                        #print "to the left"
                        #theta_r = self.theta_t - self.theta_delta
                    #self.set_wheelvel(self.fwdvel, theta_r)
        return 

    def waypoint_update(self):
        """
        update the start and target point for new segment
        Input:
            self
        Output:
            no output
        """
        self.current_start=self.current_position
        self.current_target=self.trajectory[self.counter]
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

    def compute_line(self):
        """
        Computes the parameters a,b,c for the line of the current segment
            Input:
                self: require start and target value of current segment
            Output:
                line array with a,b,c parameters
        """
        x_t=self.current_target[0]
        y_t=self.current_target[1] 
        
        x_s=self.current_start[0] 
        y_s=self.current_start[1] 

        if np.abs(x_t-x_s) < 0.000001:
            xts = 0.000001
            if x_s > x_t:
                xts = xts*(-1)
        else:
            xts = x_t-x_s

        m0=(y_t-y_s)/(xts)
        a=m0
        b=-1.
        c=y_s-x_s*m0
        self.theta_t = m.atan2(y_t-y_s,x_t-x_s)
        line=np.array([a,b,c])

        return line

    def dist_to_line(self,line):
        """
        Computes the distance between the robot and the line of the current segment
            Input:
                line: line parameters a,b,c
            Output:
                distance to line
        """
        a=line[0]
        b=line[1]
        c=line[2]

        x_r=self.current_position[0]
        y_r=self.current_position[1] 
        x_s=self.current_start[0]
        
        d=m.fabs(a*x_r+b*y_r+c)/m.sqrt(a**2+b**2)

        if a>1e6:
            d=x_r-x_s    
        return d

    def pub_wheelvel(self,wr,wl):
        wheelcmd = WheelCmdVel()
        wheelcmd.desiredWV_R = wr
        wheelcmd.desiredWV_L = wl
        self.cmd_pub.publish(wheelcmd)
        print('wheel velocities:',wr,', ', wl)
        return

    def set_wheelvel(self,vel,theta_ref):
        gain=0.1
        theta_r = self.current_position[2]
        
        phi_dot=gain*(theta_ref-theta_r)
        w_r=self.b*((self.fwdvel/self.r)+phi_dot)
        w_l=self.b*((self.fwdvel/self.r)-phi_dot)
        #if theta_r < theta_ref+self.theta_tol and theta_r > theta_ref-self.theta_tol:
            #w_r = vel/self.r
            #w_l = vel/self.r
            #print "go straight"
        #elif theta_r < theta_ref+5*self.theta_tol and theta_r > theta_ref-5*self.theta_tol:
            #delta_vel=.0015
            #if theta_r < theta_ref:
                #w_r=(vel+delta_vel)/self.r
                #w_l=(vel-delta_vel)/self.r
                #print "correcting left slowly"
            #else:
                #w_r=(vel-delta_vel)/self.r
                #w_l=(vel+delta_vel)/self.r
                #print "correcting right slowly"
        #else:
            #delta_vel=.003
            #if theta_r < theta_ref:
                ##w_r = self.turnrate/180*m.pi*self.b/self.r
                ##w_l = -w_r
                #w_r=(vel+delta_vel)/self.r
                #w_l=(vel-delta_vel)/self.r
                #print "correcting left"
            #else:
                ##w_l = self.turnrate/180*m.pi*self.b/self.r
                ##w_r = -w_l
                #w_r=(vel-delta_vel)/self.r
                #w_l=(vel+delta_vel)/self.r
                #print "correcting right"

        self.pub_wheelvel(w_r,w_l)

if __name__ == '__main__':
    rospy.init_node("navigator",anonymous=True)
    node = Navigate()
    rospy.spin()
