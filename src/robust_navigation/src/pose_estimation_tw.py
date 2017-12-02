#!/usr/bin/python

"""
2.12 Pose Estimation: Using AprilTag and Odometry to estimate the current robot position (X, T, Theta) in the world frame
The algorithm using here is sliding time window algorithm:
P_t = w*Pat_t + (1-w)*(P_(t-1) + dPod_t)
P: estimated position
Pat: estimated position from the april tag
Pod: estimated change of position from the odometry input

coordinate syntax (no specify frame, then from base_link to map):
P: (x,y,theta)
pose: (x,y,z)
angle: (x,y,z) normally only z has value
quat: (x,y,z,w)
T: homogenious matrix

PreReq:
    /apriltags/detections, /encoder_estimate_position/pose, /encoder_estimate_position/angle 
    should be published already
    tf_setup.py should be setup correctly
    The Robot should pause for one second before start moving to initialize the encoder message
Joe Huang Oct 2017
"""

import rospy
import numpy as np
import tf.transformations as tfm
from geometry_msgs.msg import Vector3
from apriltags.msg import AprilTagDetections
import tf_setup
import pdb


class Estimate_Position:
    """
    The node that estimate the position of the robot
    Subscribe topics: 
        /apriltags/detections:
        /encoder_estimate_position/pose: geometry_msgs/Vector3
        /encoder_estimate_position/angle: geometry_msgs/Vector3
    Publish topics:
        /estimate_position/pose: geometry_msgs/Vecotr3
        /estimate_position/pose_stderr: geometry_msgs/Vecotr3
        /estimate_position/angle: geometry_msgs/Vecotr3
        /estimate_position/angle_stderr: geometry_msgs/Vecotr3
    """
    def __init__(self):
        self.apriltag_sub = rospy.Subscriber("/apriltags/detections", AprilTagDetections, self.apriltag_callback, queue_size = 1)
        self.odom_pose_sub = rospy.Subscriber("/encoder_estimate_position/pose", Vector3, self.odom_pose_callback, queue_size = 1)
        self.odom_angle_sub = rospy.Subscriber("/encoder_estimate_position/angle", Vector3, self.odom_angle_callback, queue_size = 1)
        
        self.pose_pub = rospy.Publisher("/estimate_position/pose", Vector3, queue_size = 1)
        self.pose_stderr_pub = rospy.Publisher("/estimate_position/pose_stderr", Vector3, queue_size = 1)
        self.angle_pub = rospy.Publisher("/estimate_position/angle", Vector3, queue_size = 1)
        self.angle_stderr_pub = rospy.Publisher("/estimate_position/angle_stderr", Vector3, queue_size = 1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        self.apriltag_ready = False # is the apriltag ready?
        self.past_odom_ready = False # save the previous odom?
        self.odom_pose_ready = False # first time pose callback?
        self.odom_angle_ready = False # first time angle callback?
        self.predicted_pose_list = [] # the pose list got from apriltag
        self.use_err = False #CHANGED by Spencer on 19NOV... should be able to start robot anywhere now. set to TRUE to revert to old method of throwing out AT detections
        self.use_dist = True 

        self.current_odom_pose = np.array([0.25,0.5,0])
        self.current_odom_angle = np.array([0.,0.,1.57])
        self.previous_P = tf_setup.init_P_base_link_to_map 
        self.w = tf_setup.apriltag_w
        self.blm = tf_setup.Base_Link_to_Map()
        self.threshold = tf_setup.threshold
        self.threshold_dist = 2
        self.successes = np.zeros(50)
        
    def odom_pose_callback(self, data):
        """
        Record the integrated pose that the encoder outputs
        Input:
            data: Vector3; the pose of the robot calculated by Arduino
        Outpu:
            No Output
        """        
        self.current_odom_pose = np.array([data.x, data.y, data.z])
        self.odom_pose_ready = True
        return

    def odom_angle_callback(self, data):
        """
        Record the integrated angle that the encoder outputs
        Input:
            data: Vector3; the angle of the robot calculated by Arduino
        Output:
            No Output
        """        
        self.current_odom_angle = np.array([data.x, data.y, data.z])
        self.odom_angle_ready = True
        return

    def timer_callback(self, msg):
        """
        Update the pose based on stored odometry reading and stored apriltag prediction
        Input:
            msg: don't care about this
        Output:
            No Output
        """
        if self.odom_pose_ready and self.odom_angle_ready:
            # if odometry is ready
            if self.past_odom_ready:
                # if past odometry is ready
                x = self.current_odom_pose[0] - self.past_odom_pose[0]
                y = self.current_odom_pose[1] - self.past_odom_pose[1]
                theta = self.current_odom_angle[2] - self.past_odom_angle[2]
                dPod = np.array([x,y,theta])
                self.past_odom_angle = self.current_odom_angle.copy()
                self.past_odom_pose = self.current_odom_pose.copy()
                # update based on odometry
                self.previous_P = self.previous_P + dPod
                #odemetry_angle=self.previous_P[2]
            else:
                self.past_odom_ready = True
                self.past_odom_angle = self.current_odom_angle.copy()
                self.past_odom_pose = self.current_odom_pose.copy()
        if self.apriltag_ready:
            self.previous_P = (sum(self.predicted_pose_list) * self.w + self.previous_P) / (self.w * len(self.predicted_pose_list) + 1.)
            #self.previous_P[2]=odemetry_angle
            self.apriltag_ready = False

        # calculate estimated position and publish them
        self.pose_pub.publish(Vector3(self.previous_P[0], self.previous_P[1], 0.))
        self.angle_pub.publish(Vector3(0.,0.,self.previous_P[2]))
        self.pose_stderr_pub.publish(Vector3(0.,0.,0.))
        self.angle_stderr_pub.publish(Vector3(0.,0.,0.))
        return


    def apriltag_callback(self, data):
        """
        Use the apriltag location to predict the robot location and save them
        Input:
            data: AprilTagDetections; the ROS message of detected apriltags
        Output:
            No Output
        """
        # P related variables are all [X, Y, Theta]
        # sum the estimate position of robot poses from the apriltags
        predicted_pose_list = []
        for detection in data.detections:
            try:
                tag_pose = detection.pose.position
                tag_quat = detection.pose.orientation
                apriltag_id = detection.id
                pose_tag_to_camera_link = np.array([tag_pose.x, tag_pose.y, tag_pose.z])
                quat_tag_to_camera_link = np.array([tag_quat.x, tag_quat.y, tag_quat.z, tag_quat.w])
                P_predicted = self.blm.get_tf(pose_tag_to_camera_link, quat_tag_to_camera_link, apriltag_id)
                rospy.sleep(0.02)
		if self.use_err == True:
                    error = np.sqrt(np.sum(np.square(self.previous_P - P_predicted)))
                    if error <= self.threshold:
                        predicted_pose_list.append(P_predicted)
                        print "april tag ready"
                    else:
                        print error
                        print "far from initial position"
		elif self.use_dist == True:
		    dist = np.sqrt(np.square(pose_tag_to_camera_link[1])+np.square(pose_tag_to_camera_link[2]))
		    if dist <= self.threshold_dist:
                        predicted_pose_list.append(P_predicted)
                        print "april tag ready"
		    else:
                        print dist
                        print "far from available apriltag"
		else:
                    predicted_pose_list.append(P_predicted)
                    print "april tag ready"
		    
            except:
                #raise ValueError("New AprilTag Discovered! Happy Holloween!")
                print "New AprilTag Discovered! Happy Holloween!"
        if len(predicted_pose_list) != 0:
            self.predicted_pose_list = predicted_pose_list
	    self.successes = np.roll(self.successes,-1)
	    self.successes[49] = 1
            self.apriltag_ready = True
	else:
	    self.successes = np.roll(self.successes,-1)
	    self.successes[49] = 0
	print(np.mean(self.successes))
        return

if __name__ == '__main__':
    rospy.init_node("state_estimator", anonymous=True)
    node = Estimate_Position()
    rospy.spin()
    
