#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node

# Import the required modules
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose2D 

import cv2 as cv
from cv2 import aruco
from cv_bridge import CvBridge
import numpy as np

##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        self.node_name = 'ar_uco_detector'
        super().__init__(self.node_name)
        # Subscribe the topic /camera/image_raw
        self.r_image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        # Publish to topic /aruco_detection
        self.ad_publisher = self.create_publisher(Pose2D,'/detect_aruco',1)

        self.bridge = CvBridge()
        #A dictionary of 50 4x4 aruco markers
        self.marker_dict = aruco.Dictionary_get(aruco.DICT4X4_50)

        #Creating a window to display the image from the camera
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        cv.MoveWindow(self.cv_window_name, 25, 75)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Making the image as an numpy array
        # cv_image = np.array(cv_image,dtype=np.uint8)

        # Using the default parameters
        arucoParams = cv.arucoDetectorParameters_create()
        #Find all the ArUco Markers!
        corners, ids, rejected  = cv.aruco.detectMarkers(cv_image, self.marker_dict, parameters=arucoParams) 
        # DO THE CALCULATIONS HERE : EXTACT x, y & theta from the extracted data

        #Displaying the image in the window
        cv.imshow(self.cv_window_name, cv_image)

        #Detect Aruco marker
        self.get_logger().info(str(cv.aruco.detect.markers(cv_image)))


# This will return (corners, ids, rejected)

# Take the corners of the quadralateral (i.e. list of (x,y) of the four corners) of id of interest... and... extract (x,y,theta) from it 😛

        # Publish the bot coordinates to the topic  /detected_aruco
        control_pose = Pose2D()

            # Replace the 0s with the extracted x,y and theta
        control_pose.x = 0
        control_pose.y = 0
        control_pose.theta =0
        

       

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
