#!/usr/bin/python

# 2.12 Lab 3 me212bot_node: ROS driver running on the pc side to read and send messages to Arduino
# Prereq: upload controller.ino in /src to the robot and turn on the motor
# Subscribe topic: 
#   /cmdvel: WheelCmdVel
# Published topic:
#   /encoder_estimate_position/pose: geometry_msgs/Vector3
#   /encoder_estimate_position/angle: geometry_msgs/Vector3
# Peter Yu Sept 2016

import rospy
import threading
import serial
import tf.transformations as tfm
from geometry_msgs.msg import Pose, Quaternion, Vector3

from robot_base_controller.msg import WheelCmdVel

serialComm = serial.Serial('/dev/ttyACM0', 115200, timeout = 5)

## main function (Need to modify)
def main():
    rospy.init_node('me212bot', anonymous=True)
    
    odometry_thread = threading.Thread(target = read_odometry_loop)
    odometry_thread.start()
    
    ## 1. Initialize a subscriber
    cmdvel_sub = rospy.Subscriber('/cmdvel', WheelCmdVel, cmdvel_callback)
    
    
    rospy.spin()


## msg handling function (Need to modify)
def cmdvel_callback(msg):  
    ## 2. Send msg.desiredWV_R and msg.desiredWV_L to Arduino.
    serialComm.write('%f,%f\n' % (msg.desiredWV_R, msg.desiredWV_L))
    #serialComm.write('%f,%f\n' % (0.1, 0.1))
    return


# read_odometry_loop() is for reading odometry from Arduino and publish to rostopic. (No need to modify)
def read_odometry_loop():
    prevtime = rospy.Time.now()
    pose_pub = rospy.Publisher('/encoder_estimate_position/pose', Vector3, queue_size=1)
    angle_pub = rospy.Publisher('/encoder_estimate_position/angle', Vector3, queue_size=1)
    while not rospy.is_shutdown():
        # get a line of string that represent current odometry from serial
        serialData = serialComm.readline()
        
        # split the string e.g. "0.1,0.2,0.1" with cammas
        splitData = serialData.split(',')
        
        # parse the 3 split strings into 3 floats
        try:
            x     = float(splitData[0])
            y     = float(splitData[1])
            theta = float(splitData[2])
            
            hz    = 1.0 / (rospy.Time.now().to_sec() - prevtime.to_sec())
            prevtime = rospy.Time.now()
            
            print 'x=', x, ' y=', y, ' theta =', theta, ' hz =', hz
            pose = Vector3()
            pose.x = x
            pose.y = y
            angle = Vector3()
            angle.z = theta
            pose_pub.publish(pose)
            angle_pub.publish(angle)
            
        except:
            # print out msg if there is an error parsing a serial msg
            print 'Cannot parse', splitData
            

if __name__=='__main__':
    main()


