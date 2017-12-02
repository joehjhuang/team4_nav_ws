#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from robot_base_controller.msg import WheelCmdVel

class Keyboard_Control:
    def __init__(self):
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        self.cmd_pub = rospy.Publisher('/cmdvel', WheelCmdVel, queue_size = 1)
    def cmd_callback(self, msg):
        b = 0.225
        r = 0.037
        v = msg.linear.x * 0.02
        omega = msg.angular.z * 0.01
        wheelcmd = WheelCmdVel()
        wheelcmd.desiredWV_R = (omega * b + v) / r
        wheelcmd.desiredWV_L = (v - omega * b) / r
        self.cmd_pub.publish(wheelcmd)

if __name__ == '__main__':
    rospy.init_node("keyboard_control", anonymous=True)
    node = Keyboard_Control()
    rospy.spin()
