#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2B of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''


# Team ID:		2883
# Author List:  Amritanshu, Anurag, Saumitra, Ansh
# Filename:		bot_controller.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.msg import Goal  
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D 

# Matrix used for Inverse Kinematics
values = [-0.33, 0.58, 0.33, -0.33, -0.58, 0.33, 0.67, 0, 0.33]
    #Create a 3x3 numpy matrix out of this list
matrix = np.array(values)
matrix = matrix.reshape(3,3)

DEBUG = True


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Publisher and Subscriber
        # NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task

        # Bot 1 publishers:
        self.bot1_v1_publisher = self.create_publisher(Wrench,'/hb_bot_1/left_wheel_force',1)
        self.bot1_v2_publisher = self.create_publisher(Wrench,'/hb_bot_1/right_wheel_force',1)
        self.bot1_v3_publisher =self.create_publisher(Wrench,'/hb_bot_1/rear_wheel_force',1)

        # Bot 2 publishers:
        self.bot2_v1_publisher = self.create_publisher(Wrench,'/hb_bot_2/left_wheel_force',1)
        self.bot2_v2_publisher = self.create_publisher(Wrench,'/hb_bot_2/right_wheel_force',1)
        self.bot2_v3_publisher =self.create_publisher(Wrench,'/hb_bot_2/rear_wheel_force',1)

        # Bot 3 publishers:  
        self.bot3_v1_publisher = self.create_publisher(Wrench,'/hb_bot_3/left_wheel_force',1)
        self.bot3_v2_publisher = self.create_publisher(Wrench,'/hb_bot_3/right_wheel_force',1)
        self.bot3_v3_publisher =self.create_publisher(Wrench,'/hb_bot_3/rear_wheel_force',1)

	    # Initialise the required variables

        # Bot 1:
        self.bot_1_x = 0.0
        self.bot_1_y = 0.0
        self.bot_1_theta = 0.0

        # Bot 2:
        self.bot_2_x = 0.0
        self.bot_2_y = 0.0
        self.bot_2_theta = 0.0

        # Bot 3:
        self.bot_3_x = 0.0
        self.bot_3_y = 0.0
        self.bot_3_theta = 0.0


        #Similar to this you can create subscribers for hb_bot_2 and hb_bot_3
        self.bot1_subscription = self.create_subscription(
            Goal,  
            'hb_bot_1/goal',  
            self.Bot1_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.bot2_subscription = self.create_subscription(
            Goal,  
            'hb_bot_2/goal',  
            self.Bot2_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        ) 

        self.bot3_subscription = self.create_subscription(
            Goal,  
            'hb_bot_3/goal',  
            self.Bot3_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        ) 

        self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        self.Kp = 4
        self.index = 0
        self.flag = 0

    def inverse_kinematics():
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        pass

    def Bot1_CallBack(self, msg):
        self.bot_1_x = msg.x
        self.bot_1_y = msg.y
        self.bot_1_theta = msg.theta

    def Bot2_CallBack(self, msg):
        self.bot_2_x = msg.x
        self.bot_2_y = msg.y
        self.bot_2_theta = msg.theta

    def Bot3_CallBack(self, msg):
        self.bot_3_x = msg.x
        self.bot_3_y = msg.y
        self.bot_3_theta = msg.theta

def main(args=None):
    rclpy.init(args=args)
    
    hb_controller = HBController()
       
    # Main loop
    while rclpy.ok():

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
