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

import cv2.aruco
import numpy as np
from cv_bridge import CvBridge
import cv2

##############################################################


def marker_center_diag_intersection(corners):
    # Calculate midpoints of the diagonals
    diagonal1_x = (corners[0][0] + corners[2][0]) / 2
    diagonal1_y = (corners[0][1] + corners[2][1]) / 2
    diagonal2_x = (corners[1][0] + corners[3][0]) / 2
    diagonal2_y = (corners[1][1] + corners[3][1]) / 2

    # Calculate the center as the intersection of diagonals
    center_x = (diagonal1_x + diagonal2_x) / 2
    center_y = (diagonal1_y + diagonal2_y) / 2

    return center_x, center_y

def marker_center_coord_avg(corners):
    # Calculate the average of X and Y coordinates
    x_sum = sum(point[0] for point in corners)
    y_sum = sum(point[1] for point in corners)
    center_x = x_sum / len(corners)
    center_y = y_sum / len(corners)
    return center_x, center_y


class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
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

        #Detect Aruco marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_parameters)

        # Print detected aruco id with its corners
        for id, corner in zip(ids, corners):
            corner = corner[0]
            diag_intersection = marker_center_diag_intersection(corner)
            avg_intersection = marker_center_coord_avg(corner)
            print(id[0], diag_intersection, avg_intersection)
        print()

        # Display the image with aruco marers using OpenCV
        image_with_markers = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        # cv2.imshow('Camera Image', image_with_markers)
        # cv2.waitKey(1)

        # Publish the bot coordinates to the topic  /detected_aruco
        # control_pose = Pose2D()

            # Replace the 0s with the extracted x,y and theta
        # control_pose.x = 0
        # control_pose.y = 0
        # control_pose.theta =0
        


def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
