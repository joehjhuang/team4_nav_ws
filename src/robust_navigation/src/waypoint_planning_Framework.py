#!/usr/bin/python

"""
2.12 Navigation Hard-Coded Waypoints based on Initial Determination of Object Location
Replace with DWA local planning when hard-code (Level 1) navigation is complete

1) Determine which of two possible arena layouts the robot is facing
2) Based on which arena layout it is, publish a topic as an array of waypoints that is subscribed to by Nav_state.py

3#) Ensure that nav_state successfully subscribes to and implements the waypoints
4#) Break down waypoints by task so that robot can start from new tasks to create basic redundancy
5###) Add DWA as a means to try to get to the objective if all else fails

"""

"""

NEW PUBLISHED TOPIC: "current_waypoint"

This code is constantly interacting through ROS with nav_state
At any given time:
	waypoint_planning --> current_waypoint --> nav_state
	waypoint_planning <-- counter (topic is nav_counter) <-- nav_state
	
Only during detectObjects()
	waypoint_planning --> task (which color) --> object_detection

Currently, the code just has two sets of waypoints saved. It will first determine which course of action to take based on an object detection function
and then it will send waypoints to nav_state until nav_state says it has reached that waypoint, at which time nav_state updates its counter, which gets 
sent back to waypoint_planning, and waypoint_planning starts sending the next waypoint.

"""
"""
Ideally, we still need a basic navigation fallback to reach objects in case this waypoint system fails.
It would need to call an objective from somewhere else (so there would be a few different possible objectives that are human inputted as part of a topic
and then it would use a basic algorithm like DWA to get itself to the target

"""

import rospy
import numpy as np
import math as m
import std_msgs.msg
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16
#from trajectory_feeder import feeder
#from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
#print(geometry_msgs.msg.__file__)

class setWaypoint:
	def __init__(self):
		
		self.continuous_Navigation = False
		#self.destination = rospy.Subscriber("/destination",Vector3,self.destination_callback,queue_size=1)
		
		### Callback the current position and angle
		self.position=np.array([0.0,0.0,0.0])
		self.pose_sub=rospy.Subscriber("/estimate_position/pose",Vector3,self.pose_callback,queue_size=1)
		self.angle_sub=rospy.Subscriber("/estimate_position/angle",Vector3,self.angle_callback,queue_size=1)
		
		### Callback the angle from nav_state
		self.counter_sub=rospy.Subscriber("/nav_counter",Vector3,self.counter_callback,queue_size=1)
		
		### Callback the location of the biggest object that it sees from object_detection
		self.objectloc = np.array([0.0,0.0,0.0])
		self.objectloc_sub = rospy.Subscriber("/object_location/global",Vector3,self.objectloc_callback,queue_size=1)
		
		### Makes an internal counter
		self.internal_counter = 0
		self.counter = 0
		
		### Define a publisher to print out the task for object_detection
		self.pub = rospy.Publisher('/current_task', Int16, queue_size=10)
		self.objectloc_sub = rospy.Subscriber("/object_location/global",Vector3,self.objectloc_callback,queue_size=1)
        
		### Define the red boxes, will expand this to include any other collision objects
		"Note: Take measurements and record corners for the red boxes"
		self.collisionObject_red1 = [[0,0],[0,0],[0,0],[0,0]]
		self.collisionObject_red2 = [[0,0],[0,0],[0,0],[0,0]]
		
		### Define the potential green and purple boxes both as false
		self.collisionObject_green = False
		self.collisionObject_purple = False
		
		self.waypoint_pub = rospy.Publisher("/current_waypoint",Vector3,queue_size=1)
		
		### Here is the set of waypoints to follow if the purple obstacle is there
		self.purpleCase_waypointArray = [[0.35,1.2],[1.32,1.2],[1.32,2.12]]
		
		### Here is the set of waypoints to follow if the green obstacle is there
		self.greenCase_waypointArray = [[0.28,1.62],[1.32,1.62],[1.32,2.12]]
		
		### The boolean will run detectObjects until it finds the proper location of objects,
		### Then it will operate based on a hardcoded set of waypoints
		
			
	def pose_callback(self,data):
		self.position[0]=data.x
		self.position[1]=data.y
		return
		
	def angle_callback(self,data):
		self.position[2]=data.z
		
		if self.continuous_Navigation == False:
			if self.collisionObject_purple == True:
				print "Assuming a purple collision object"
				self.publish_currentWaypoint(self.purpleCase_waypointArray[self.counter])
				return
			elif self.collisionObject_green == True:
				print "Assuming a green collision object"
				self.publish_currentWaypoint(self.greenCase_waypointArray[self.counter])
				return
			else:
				print "Finding Objects"
				self.detectObjects()
				return
		else:
			### Run the continuous route planning here.
			# Each c-llision object would be represented as a 4 coordinate rectangle, with each coordinate representing
			# [upper left, upper right, lower left, lower right]
			# These would be all be appended to a collision_object list
			# DONT" FORGET THAT THIS STILL NEEDS A CALLBACK FUNCTION FOR DESTINATION AND SOME CODE TO ACTUALLY GENERATE THAT DESTINATION....
			# UNLESS: we could set a number of destinations that pre-exist in this code, and nav_state could simply track that we get there
			# We would use the location of robot, phi of robot, and location of destination to create a set of points some radius away from the robot
			# We would then eliminate invalid points by looping through and checking whether they would make the robot hit any obstacle
					# This can be done either by making the points a short distance away, or by drawing a line (or two lines) from the robot to the target
					# and checking whether that line intersects any other lines
			# Then we would assign each point a value by how close it is to the target, and potentially how good the next set of points is (so that you don't move 		# right up to an obstacle but instead work your way around it
			return
		
		return
		
	def objectloc_callback(self,data):
		self.objectloc = [data.x,data.y,data.z]
        self.task_complete = True
		return
		
	def counter_callback(self,data):
		self.counter = data.x
		return
		
	def publish_currentWaypoint(self,current_waypoint):
		self.waypoint_pub.publish(Vector3(current_waypoint[0],current_waypoint[1],current_waypoint[2]))
		return
		
	def task_publisher(self,task):
		self.pub.publish(Int16(task))
        self.task_complete = False
		return

	def detectObjects(self):
		self.task_publisher(3)
        
        while not self.task_complete:
            rospy.sleep(0.1)
            
		rho = m.sqrt(self.objectloc[0]**2 + self.objectloc[1]**2)
		print "rho of green= ", rho
		"Find the actual value for rho necessary to ensure it knows which one is correct and Put it Here"
		if rho <= 1.6 and rho >= 1.4:
			self.collisionObject_green = True
			return
		self.task_publisher(1)
		rho = m.sqrt(self.objectloc[0]**2 + self.objectloc[1]**2)
		print "rho of purple= ", rho
		"Find the actual value for rho necessary to ensure it knows which one is correct and Put it Here"
		if rho <= 1.6 and rho >= 1.3:
			self.collisionObject_green = True
			return
		return
		

if __name__ == '__main__':
	rospy.init_node("waypoint_planner",anonymous=True)
	#fdr = feeder()
	node = setWaypoint()
	rospy.spin()
