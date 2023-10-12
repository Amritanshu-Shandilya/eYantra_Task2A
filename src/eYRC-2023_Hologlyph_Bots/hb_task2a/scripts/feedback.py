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
from geometry_msgs.msg import Pose2D #verification required

import cv2 as cv
from cv_bridge import CvBridge
import numpy as np

##############################################################
class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        # Subscribe the topic /camera/image_raw
        self.r_image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        # Publish to topic /aruco_detection
        self.create_publisher(Pose2D,'/detect_aruco',1)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)


    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


        #Detect Aruco marker
        self.get_logger().info(str(cv.aruco.detect.markers(cv_image)))

        # Publish the bot coordinates to the topic  /detected_aruco
        

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
