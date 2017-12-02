#!/usr/bin/python
import rospy
from std_msgs.msg import Bool	

class worker:
	def __init__(self):
		self.completion_sub = rospy.Subscriber("/phase_completion",Bool,self.completion_callback,queue_size=5)
		self.completion_pub = rospy.Publisher("/phase_completion",Bool,queue_size=5)
		self.task_complete = 1
		rospy.init_node('worker')

	def completion_callback(self,data):
		print('received a topic update!')
		self.task_complete = data.data

		print('internalized topic update!')
		print(self.task_complete)

		if self.task_complete:
			print('Reading True!')
		else:
			print('Reading False!')
			self.completion_pub.publish(True)

if __name__ == "__main__":
	print('running main script for worker node!')
	worker1 = worker()
	rospy.spin()
