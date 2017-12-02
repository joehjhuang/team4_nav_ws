#!/usr/bin/python
import rospy
from std_msgs.msg import Bool, String

class worker:
	def __init__(self):
		rospy.init_node('taskmaster')
		self.task_completion = rospy.Publisher("/task_completion",Bool,queue_size=10)
		self.current_phase   = rospy.Publisher("/current_phase",String,queue_size=10)
		
