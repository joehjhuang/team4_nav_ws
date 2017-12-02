#!/usr/bin/python

"""
2.12 Task Switcher: Detect Keypress and published it as task number
Note: publishing task number frequently, before first pressed, it shows 100. Press <Esc> to escape keyboard detection

PreReq:
    Fill in the "TASK_LIST" below to show all the task numbers can be chosen from, no more than 10 tasks, do not put <Esc> as task number
Joe Huang Nov 2017
"""
import rospy
from std_msgs.msg import Int16
import tty
import sys
import termios

TASK_LIST = [1,2,3,4,5,6,7]

class Task_Switcher:
    def __init__(self):
        """
        The node that show which task should operate
        Publish topics:
            /current_task: std_msgs/Int16
        """
        print "press task number, press <Esc> to escape keyboard detecting mode"
        orig_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin)
        self.task = 100
        self.pub = rospy.Publisher('/current_task', Int16, queue_size=10)
        rospy.Timer(rospy.Duration(0.02), self.callback)
        while True:
            key = ord(sys.stdin.read(1)[0]) - 48
            if key in TASK_LIST:
                self.task = key
            if key == -21:
                print "exit keyboard monitor"
                break
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

    def callback(self, msg):
        task = Int16(self.task)
        self.pub.publish(task)
        return

if __name__ == '__main__':
    rospy.init_node('task_switcher')
    node = Task_Switcher()
    rospy.spin()
