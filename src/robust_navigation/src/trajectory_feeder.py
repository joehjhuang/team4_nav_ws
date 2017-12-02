#!/usr/bin/python

import rospy
import numpy as np 
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class feeder:
	def __init__(self):
		rospy.init_node('feeder',anonymous=True)
		self.arrived_waypoint_sub = rospy.Subscriber("/arrived_waypoint",Bool,self.arrived_callback,queue_size=1)
		self.arrived_waypoint_pub = rospy.Publisher("/arrived_waypoint",Bool,queue_size=1)
		self.waypoint_pub         = rospy.Publisher("/current_waypoint",Vector3,queue_size=1)
		self.arrived_dest_pub     = rospy.Publisher("/arrived_waypoint",Bool,queue_size=1)

	def feed(self, traj):
			self.traj = traj
			self.num_waypoints = traj.shape[0]
			self.counter = 0
			self.publish_waypoint(self.traj[0])

	def arrived_callback(self,data):
		if data.data:
			self.counter = self.counter + 1
			if self.counter >= self.num_waypoints-1:
				self.arrived_dest_pub.publish(True)
			else:
				self.publish_waypoint(traj[self.counter])
			print('trajectory_feeder knows you arrived!')

	def publish_waypoint(self,waypoint):
		data = Vector3(waypoint[0],waypoint[1],0)
#		data.x = waypoint[0]
#		data.y = waypoint[1]
#		data.z = 0
		print('publishing waypoint!')
		self.waypoint_pub.publish(data)
		print('published waypoint!')

if __name__ == '__main__':
	traj = np.array([[0.,0.,0.],[1.,1.,1.],[2.,2.,2.]])
	fdr = feeder()
	rospy.sleep(2)
	fdr.feed(traj)
	rospy.sleep(1)
	rospy.spin()
