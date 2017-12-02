#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for detecting objects
# Peter Yu Oct 2016

import rospy
import numpy as np
from numpy.linalg import inv
import cv2  # OpenCV module

from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, Twist, Vector3, Quaternion
#print(geometry_msgs.msg.__file__)
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int16

from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math
    
class Object_Detection:
    def __init__(self):
        # Publisher for publishing pyramid marker in rviz
        #self.pyramid_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10) 
        
        # Publisher for publishing images in rviz
        #self.cross_pub = rospy.Publisher('/object_detection/image_with_cross', Image, queue_size=10)
        #self.eroded_pub = rospy.Publisher('/object_detection/mask_eroded', Image, queue_size=10)
        #self.diluted_pub = rospy.Publisher('/object_detection/mask_eroded_dilated', Image, queue_size=10)
        #self.result_pub = rospy.Publisher('/object_detection/img_result', Image, queue_size=10)
        
        self.pose_sub=rospy.Subscriber("/estimate_position/pose",Vector3,self.pose_callback,queue_size=1)
        self.angle_sub=rospy.Subscriber("/estimate_position/angle",Vector3,self.angle_callback,queue_size=1)
        self.task_sub=rospy.Subscriber('/current_task', Int16,self.task_callback, queue_size=1)
        
        self.object_location_global_pub=rospy.Publisher("/object_location/global",Vector3,queue_size=1)
        self.object_location_base_pub=rospy.Publisher("/object_location/base",Vector3,queue_size=1)

        
        # Bridge to convert ROS Image type to OpenCV Image type
        self.cv_bridge = CvBridge()  
        
        # Get the camera calibration parameter for the rectified image
        msg = rospy.wait_for_message('/camera/rgb/camera_info', CameraInfo, timeout=None) 
        #     [fx'  0  cx' Tx]
        # P = [ 0  fy' cy' Ty]
        #     [ 0   0   1   0]
        
        self.fx = msg.P[0]
        self.fy = msg.P[5]
        self.cx = msg.P[2]
        self.cy = msg.P[6]
        
        self.task=4
        
        self.position=np.array([0.0,0.0,0.0])
    
        self.useHSV   = True
        self.useDepth = True
        
        if self.useHSV:
            if self.useDepth:
                #Use Kinect depth data
                #Subscribe to both RGB and Depth images with a Synchronizer
                self.image_sub = message_filters.Subscriber("/camera/rgb/image_rect_color", Image)
                self.depth_sub = message_filters.Subscriber("/camera/depth_registered/image", Image)
    
                #only do rosRGBCallBack when self.image_sub and self.depth_sub have same timestamp
                self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.depth_sub], 10, 0.5)
                self.ts.registerCallback(self.showDepth_callback)
            else:
                # Subscribe to RGB images
                self.image_sub=rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.showHSV_callback)
        else:
            # subscribe to image
            self.image_sub=rospy.Subscriber('/camera/rgb/image_rect_color', Image, self.showImage_callback)

    #update position from estimate
    def pose_callback(self,data):
        self.position[0]=data.x
        self.position[1]=data.y
        return
        
    #update position from estimate
    def angle_callback(self,data):
        self.position[2]=data.z
        return
        
    def task_callback(self,data):
        self.task=data.data
        return
        
    # if self.useHSV = false and self.useDepth = false
    def showImage_callback(self,data):
        # 1. convert ROS image to opencv format
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
    
        # 2. visualize it in a cv window
        cv2.imshow("OpenCV_View", cv_image)
        cv2.waitKey(3)  # milliseconds
    
    
    # if self.useHSV = true and self.useDepth = false
    def showHSV_callback(self,data):
        try:
            # 1. convert ROS image to opencv format
            cv_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        if self.task==100:
            return
            
        contours, mask_image = self.HSVObjectDetection(cv_image)
        
        for cnt in contours:
            # Find a bounding box of detected region
            #  xp, yp are the coordinate of the top left corner of the bounding rectangle
            #  w, h are the width and height of the bounding rectangle
            xp,yp,w,h = cv2.boundingRect(cnt)  
            
            # Draw the bounding rectangle
            if not (((xp+w/2) == 320) and ((yp+h/2) == 240)):
                cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255], 2)
            
                centerx, centery = xp+w/2, yp+h/2
                print "rectangle center", centerx, centery
            
        #cv2.imshow("OpenCV_View", cv_image)
        #cv2.waitKey(3)  # milliseconds
        
    # HSV filtering and contour detection
    def HSVObjectDetection(self, cv_image):
        # convert image to HSV color space
        hsv_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        self.task_sub=rospy.Subscriber('/current_task', Int16,self.task_callback, queue_size=1)
        print "looking for ", self.task

        if self.task==1:
            # define range of red color in HSV
            #lower = np.array([5,0,0])
            #upper = np.array([175,255,255])
            lower = np.array([100,50,50])
            upper = np.array([180,255,255])
            
        elif self.task ==2:
            # define range of green color in HSV
            lower = np.array([50,0,0])
            upper = np.array([80,255,255])
        
        elif self.task ==3:
            # define range of blue color in HSV
            lower = np.array([110,0,0])
            upper = np.array([120,255,255])
        
        elif self.task ==4:
            # define range of yellow color in HSV
            lower = np.array([20,0,0])
            upper = np.array([30,255,255])
        
        ### This automatically searches for red, works for brightly lit conditions and red
        else:
            lower = np.array([100,50,50])
            upper = np.array([180,255,255])
    
        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image,lower,upper)
        #if self.task==1:
            #mask = cv2.bitwise_not(mask) #only required if red bottle is chosen
        
        #cv2.imshow("HSV /w threshold", mask)       
        #cv2.waitKey(3)  # milliseconds
        
        #erode and dilate image
        mask_eroded = cv2.erode(mask,None,iterations=2)
        #cv2.imshow("HSV eroded", mask_eroded)
        #cv2.waitKey(3)  # milliseconds
        
        mask_eroded_dilated = cv2.dilate(mask_eroded,None,iterations=10)
        #cv2.imshow("HSV eroded and dilated", mask_eroded_dilated)
        #cv2.waitKey(3)  # milliseconds
            
        image,contours,hierarchy = cv2.findContours(mask_eroded_dilated,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        return contours, mask_eroded_dilated
    
    # if self.useHSV = true and self.useDepth = true
    def showDepth_callback(self, rgb_data, depth_data):
        r=rospy.Rate(10)
        try:
            #transform Kinect image and depth info to CV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(rgb_data, "bgr8")
            cv_depthimage = self.cv_bridge.imgmsg_to_cv2(depth_data, "32FC1")
            cv_depthimage2 = np.array(cv_depthimage, dtype=np.float32)
        except CvBridgeError as e:
            print(e)

        contours, mask_image = self.HSVObjectDetection(cv_image)
        
        area_temp=0
        x_temp=0
        y_temp=0
        w_temp=0
        h_temp=0
        has_rectangle=False
        for cnt in contours:
            #read out information from bounding rectangle
            x_temp,y_temp,w_temp,h_temp = cv2.boundingRect(cnt)
            #check if rectangle in area
            if x_temp+w_temp/2 >= cv_depthimage2.shape[1] or y_temp+h_temp/2 >= cv_depthimage2.shape[0]:
                continue
            #if ((x_temp == 320) and (y_temp == 240)):
            #    continue
            #only keep rectangle with largest area
            if w_temp*h_temp > area_temp:
                xp=x_temp
                yp=y_temp
                h=h_temp
                w=w_temp
                area_temp=w*h
                has_rectangle=True 
                
        if has_rectangle == False:
            print "No object found"
            self.pub_object_location_global(np.array([0.0,0.0,0.0]))
            return
            
        #get center of surrounding rectangle    
        centerx, centery = xp+w/2, yp+h/2
        cv2.rectangle(cv_image,(xp,yp),(xp+w,yp+h),[0,255,255], 2)
        #cv2.imshow("OpenCV_View", cv_image)
        #cv2.waitKey(3)  # milliseconds
            
        #get depth value from image via averaging 
        zc=self.get_depth(centerx,centery,w,h,cv_depthimage2)
        
        #compute coordinates of object in meters in camera frame   
        if not zc == 0:
            xc = self.getXYZ(centerx, centery, zc, self.fx,self.fy,self.cx,self.cy)
            obj2global = self.getGlobalCoordinates(xc)
            obj2base = np.array([zc-0.3,centerx,centery])
        else:
            obj2global=np.array([0.0,0.0,0.0])
            obj2base=np.array([0.0,0.0,0.0])

        #print "global coordinates", obj2global
        
        #for viewing below
        #cv2.imshow("OpenCV_View", cv_image)
        #cv2.waitKey(3)  # milliseconds
        print "depth of object is: ", zc 
            
        self.pub_object_location_global(obj2global)
        self.pub_object_location_base(obj2base)

        r.sleep()
    
        
    def get_depth(self,centerx,centery,w,h,cv_depthimage2):
        sum_temp=0.0
        counter_temp=0.0
        width=10
        for i in range(0,width):
            for j in range(0,width):
                if math.isnan(cv_depthimage2[int(centery-width/2+j)][int(centerx-width/2+i)]):
                    continue
                else:
                    sum_temp = sum_temp + cv_depthimage2[int(centery-width/2+j)][int(centerx-width/2+i)]
                    counter_temp = counter_temp + 1.0
        if counter_temp==0:
            zc=0
        else:
            zc=sum_temp/counter_temp
        return zc
        
            
    ### Function to convert coordinates in camera frame to global coordinates in base frame
    def getGlobalCoordinates(self,xc):
        ### we will return the coordinate of the center of the object (in xyz) with respect to the base frame
        
        #robot dimensions
        x_cam2base=-0.3
        z_cam2base=0.3
        z_base2global = 0.05
        
        #load current robot position in global frame
        #TODO: uncomment these lines and comment the ones below
        #x_base2global = self.position[0]
        #y_base2global = self.position[1]
        #theta_base2global = self.position[2]
        
        x_base2global = 0
        y_base2global = 0
        theta_base2global = math.pi/2
        
        
        # separate the x y and z coords of object in camera frame for clarity
        #Note: for camera frame --> x is to left of cam, z is principal axis (distance in front), y is up
        x_obj2cam = xc[0]
        y_obj2cam = xc[1]
        z_obj2cam = xc[2]
        
        # use x y and z of object in camera frame to create a radius to the object and a theta between the principal axis of the robot and the imaginary line between obj and Camera in the x z plane (2d)
        r_obj2cam = math.sqrt(x_obj2cam**2 + z_obj2cam**2)
        theta_obj2cam = np.arctan2(x_obj2cam,z_obj2cam)
        
        #find coordinates of object in base frame
        x_obj2base = r_obj2cam*math.cos(theta_obj2cam) + x_cam2base
        y_obj2base = -r_obj2cam*math.sin(theta_obj2cam)
        z_obj2base = y_obj2cam + z_cam2base
        
        #find coordinates of object in the global frame
        x_obj2global = x_obj2base * math.cos(theta_base2global) - y_obj2base * math.sin(theta_base2global) + x_base2global
        y_obj2global = x_obj2base * math.sin(theta_base2global) + y_obj2base * math.cos(theta_base2global) + y_base2global
        z_obj2global = z_obj2base + z_base2global
        
        #create array of base2obj coords called x_b2o
        obj2global = np.array(([x_obj2global],[y_obj2global],[z_obj2global]))

        #return x y and z
        return obj2global
        
    #transforms pixel values into meters
        #x is to left of camera
        #y is up
        #z is away from camera
    def getXYZ(self,x, y, zc, fx,fy,cx,cy):
        
        temp = np.array(([x],[y],[1]))
        # Homogenous Transform matrix
        K = np.matrix([(fx, 0, cx),(0, fy, cy),(0, 0, 1)])
        xn=inv(K)*temp
         
        xc = zc*xn
        xc[0]=-xc[0]
        xc[1]=-xc[1]
        return xc
        
    def pub_object_location_global(self,obj2global):
        self.object_location_global_pub.publish(Vector3(obj2global[0],obj2global[1],obj2global[2]))
        return
        
    def pub_object_location_base(self,obj2global):
        self.object_location_base_pub.publish(Vector3(obj2global[0],obj2global[1],obj2global[2]))
        return
    
if __name__=='__main__':
    rospy.init_node('object_detection', anonymous=True)
    node=Object_Detection()
    rospy.spin()

        
    
