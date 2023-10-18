
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
"""Utility Functions"""
def find_Center_of_Markers(MarkerArray):
    c1, c2, c3, c4 = list(MarkerArray[0][0]), list(MarkerArray[0][1]), list(MarkerArray[0][2]), list(MarkerArray[0][3])
    # print("c1 : "+str(c1)+'\n')
    # We know that c1, c2, c3 and c4 are clockwise from top left
    diagonal1_x = (c1[0]+c3[0]) / 2
    diagonal1_y = (c1[1] + c3[1]) / 2
    diagonal2_x = (c2[0] + c4[0]) / 2
    diagonal2_y = (c2[1] + c4[1]) / 2

    # Calculate the center as the intersection of diagonals
    center_x = (diagonal1_x + diagonal2_x) / 2
    center_y = (diagonal1_y + diagonal2_y) / 2

    return (center_x, center_y)


##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        self.node_name = 'ar_uco_detector'
        super().__init__(self.node_name)
        # Subscribe the topic /camera/image_raw
        self.r_image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        # Publish to topic /aruco_detection
        self.ad_publisher = self.create_publisher(Pose2D,'/detect_aruco',1)

        
        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        self.bridge = CvBridge()


    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_250)
        parameters =  cv.aruco.DetectorParameters()
        detector = cv.aruco.ArucoDetector(dictionary, parameters)
        markers, ids, rejectedCandidates = detector.detectMarkers(cv_image)
        #Find the markers
        image_with_markers = cv.aruco.drawDetectedMarkers(cv_image, markers, ids)
        cv.imshow('Camera Image', image_with_markers)
        cv.waitKey(1)       


        # DO THE CALCULATIONS HERE : EXTACT x, y & theta from the extracted data

        # Finding the centers of the markers at the corners
        # Need to do it one time only
        #then we can use this centers list for all calculations
        centers=[]
        
        for marker in markers:
                centers.append(find_Center_of_Markers(marker))
            
        #Remove the center of bot which is at the end of the list centers
        bot_coordinates = centers.pop()
        print("Bot coordinates :  "+str(bot_coordinates))
        print("Centers :  "+str(centers))

            
            
        


        # Publish the bot coordinates to the topic  /detected_aruco
        # control_pose = Pose2D()

        #     # Replace the 0s with the extracted x,y and theta
        # control_pose.x = 0.
        # control_pose.y = 0.
        # control_pose.theta =0.
        

       

def main(args=None):
    # print("I am here")
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
