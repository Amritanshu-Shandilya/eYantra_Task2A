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
import math

##############################################################

DEBUG = False

def marker_orientation(corners):
    tl = corners[0]  # top left
    tr = corners[1]  # top right
    br = corners[2]  # bottom right
    bl = corners[3]  # bottom left
    top = (tl[0] + tr[0]) / 2, (tl[1] + tr[1]) / 2
    centre = (tl[0] + tr[0] + bl[0] + br[0]) / 4, (tl[1] + tr[1] + bl[1] + br[1]) / 4

    angle = math.atan2(top[0] - centre[0], centre[1] - top[1])
    
    # Ensure the angle is in the range [0, 2π)
    if angle < 0:
        angle += 2 * math.pi

    return angle


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


class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        # Subscribe the topic /camera/image_raw
        self.r_image_subscriber = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 1)
        # Publish to topic /aruco_detection
        self.ad_publisher1 = self.create_publisher(Pose2D,'/detect_aruco_1',10)
        self.ad_publisher2 = self.create_publisher(Pose2D,'/detect_aruco_2',10)
        self.ad_publisher3 = self.create_publisher(Pose2D,'/detect_aruco_3',10)

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)
        self.bridge = CvBridge()

        self.corners = [None, None, None, None]
        self.bot1_pos = None
        self.bot2_pos = None
        self.bot3_pos = None

        self.timer = self.create_timer(0.5, self.bot_pos_publish)


    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        #Detect Aruco marker
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        aruco_parameters = cv2.aruco.DetectorParameters()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(cv_image, aruco_dict, parameters=aruco_parameters)

        aruco_marker = {}

        # Print detected aruco id with its corners
        for id, corner in zip(ids, corners):
            corner = list(map(list, corner[0]))
            center = marker_center_diag_intersection(corner)
            angle = marker_orientation(corner)
            cv_image = cv2.circle(cv_image, (int(center[0]), int(center[1])), radius=3, color=(154, 54, 179), thickness=-1)
            aruco_marker[id[0]] = [center, angle, corner]

        if 1 in aruco_marker:
            self.bot1_pos = {"x":aruco_marker[1][0][0], "y":aruco_marker[1][0][1], "theta":aruco_marker[1][1]}
        if 2 in aruco_marker:
            self.bot2_pos = {"x":aruco_marker[2][0][0], "y":aruco_marker[2][0][1], "theta":aruco_marker[2][1]}
        if 3 in aruco_marker:
            self.bot3_pos = {"x":aruco_marker[3][0][0], "y":aruco_marker[3][0][1], "theta":aruco_marker[3][1]}
        if 6 in aruco_marker:
            self.corners[0] = aruco_marker[6][2][0]
        if 10 in aruco_marker:
            self.corners[1] = aruco_marker[10][2][1]
        if 12 in aruco_marker:
            self.corners[2] = aruco_marker[12][2][2]
        if 4 in aruco_marker:
            self.corners[4] = aruco_marker[4][2][3]

        if DEBUG:
            print(f"Corners: {self.corners}")
            print(f"Bot: Center:\t({self.bot_x}, {self.bot_y})\tTheeta:{self.bot_theeta}")


        # Display the image with aruco marers using OpenCV
        image_with_markers = cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        cv2.imshow('Camera Image', image_with_markers)
        cv2.waitKey(1)

    def bot_pos_publish(self):
        msg = Pose2D()
        print("\n")
        if not self.bot1_pos is None:
            msg.x = self.bot1_pos["x"]
            msg.y = self.bot1_pos["y"]
            msg.theta = self.bot1_pos["theta"]
            self.ad_publisher1.publish(msg)
            self.get_logger().info(f'BOT1: ({msg.x},{msg.y}), {msg.theta}')

        if not self.bot2_pos is None:
            msg.x = self.bot2_pos["x"]
            msg.y = self.bot2_pos["y"]
            msg.theta = self.bot2_pos["theta"]
            self.ad_publisher2.publish(msg)
            self.get_logger().info(f'BOT2: ({msg.x},{msg.y}), {msg.theta}')

        if not self.bot3_pos is None:
            msg.x = self.bot3_pos["x"]
            msg.y = self.bot3_pos["y"]
            msg.theta = self.bot3_pos["theta"]
            self.ad_publisher3.publish(msg)
            self.get_logger().info(f'BOT3: ({msg.x},{msg.y}), {msg.theta}')

def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
