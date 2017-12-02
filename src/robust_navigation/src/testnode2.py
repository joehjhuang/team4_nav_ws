#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

class commander:
        def __init__(self):
                self.completion_sub = rospy.Subscriber("/phase_completion",Bool,self.completion_callback,queue_size=5)
                self.completion_pub = rospy.Publisher("/phase_completion",Bool,queue_size=5)
                self.task_complete = True
                rospy.init_node('commander')

        def completion_callback(self,data):
                self.task_complete = data.data
		
                if self.task_complete == False:
                        print('task in progress')
                if self.task_complete == True:
                        print('!')

        def set_task(self):
		print('inside set_task function of commander1!')
                self.completion_pub.publish(False)

if __name__ == "__main__":
	print('running main script for commander node!')
        cdr1 = commander()
	print('commander1 initialized! sending task command')
        cdr1.set_task()	

	while not cdr1.task_complete:
		print('waiting for task completion!')
		rospy.sleep(0.001)
	
	print('continuing with execution!')	
	rospy.spin()
