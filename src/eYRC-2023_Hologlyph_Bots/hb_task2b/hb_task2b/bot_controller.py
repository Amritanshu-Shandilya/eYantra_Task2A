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
        self.bot1_v1_publisher = self.create_publisher(Wrench,'/hb_b1/left_wheel_force',1)
        self.bot1_v2_publisher = self.create_publisher(Wrench,'/hb_b1/right_wheel_force',1)
        self.bot1_v3_publisher =self.create_publisher(Wrench,'/hb_b1/rear_wheel_force',1)

        # Bot 2 publishers:
        self.bot2_v1_publisher = self.create_publisher(Wrench,'/hb_b2/left_wheel_force',1)
        self.bot2_v2_publisher = self.create_publisher(Wrench,'/hb_b2/right_wheel_force',1)
        self.bot2_v3_publisher =self.create_publisher(Wrench,'/hb_b2/rear_wheel_force',1)

        # Bot 3 publishers:  
        self.bot3_v1_publisher = self.create_publisher(Wrench,'/hb_b3/left_wheel_force',1)
        self.bot3_v2_publisher = self.create_publisher(Wrench,'/hb_b3/right_wheel_force',1)
        self.bot3_v3_publisher =self.create_publisher(Wrench,'/hb_b3/rear_wheel_force',1)

	    
        # VARIABLES TO HOLD POSITIONS
        # Bot 1:
        self.b1_x = 0.0
        self.b1_y = 0.0
        self.b1_theta = 0.0

        # Bot 2:
        self.b2_x = 0.0
        self.b2_y = 0.0
        self.b2_theta = 0.0

        # Bot 3:
        self.b3_x = 0.0
        self.b3_y = 0.0
        self.b3_theta = 0.0

        # VARIABLES TO HOLD WHEEL VELOCITIES
        # Bot 1:
        self.b1_v1 = 0.0
        self.b1_v2 = 0.0
        self.b1_v3 = 0.0

        # Bot 2:
        self.b2_v1 = 0.0
        self.b2_v2 = 0.0
        self.b2_v3 = 0.0

        # Bot 3:
        self.b3_v1 = 0.0
        self.b3_v2 = 0.0
        self.b3_v3 = 0.0

        #SUBSCRIBERS
        self.bot1_subscription = self.create_subscription(
            Goal,  
            'hb_b1/goal',  
            self.Bot1_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        )  

        self.bot2_subscription = self.create_subscription(
            Goal,  
            'hb_b2/goal',  
            self.Bot2_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        ) 

        self.bot3_subscription = self.create_subscription(
            Goal,  
            'hb_b3/goal',  
            self.Bot3_CallBack,  # Callback function to handle received messages
            10  # QoS profile, here it's 10 which means a buffer size of 10 messages
        ) 

        self.subscription  # Prevent unused variable warning

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        #For Testing purpose
        self.timer = self.create_timer(0.5, self.inverse_kinematics)

        # P-controllers for each bot
        self.Kp1 = 4
        self.Kp2 = 4
        self.Kp3 = 4

        # Indices for each bot
        self.index1 = 0
        self.index2 = 0
        self.index3 = 0

        # Flags for each bot
        self.flag1 = 0
        self.flag2 = 0
        self.flag3 = 0

    def inverse_kinematics(self):
        # goals
        bot1_x_goal =[175, 125, 125, 175][self.index1]
        bot1_y_goal = [50, 50, 100, 100 ][self.index1]
        bot1_theta_goal = [0, math.pi/2, -math.pi, -math.pi/2, 0][self.index1]

        bot2_x_goal =[200, 175, 175, 125, 225][self.index2]
        bot2_y_goal = [400, 400, 350, 350, 400][self.index2]
        bot2_theta_goal = [0, math.pi/2, -math.pi, -math.pi/2, 0][self.index2]

        bot3_x_goal =[300, 350, 350, 300][self.index3]
        bot3_y_goal = [50, 50, 100, 100][self.index3]
        bot3_theta_goal = [0, math.pi/2, -math.pi, -math.pi/2, 0][self.index3]

        # CALCULATION FOR BOT 1
        bot1_pos = [self.b1_x, self.b1_y, self.b1_theta]
        # Finding errors
        b1_x_err = bot1_x_goal - self.b1_x
        b1_y_err = bot1_y_goal - self.b1_y
        b1_theta_err = bot1_theta_goal - self.b1_theta

        # Frame changing using rotation matrix
        bot1_real_theta = -self.hb_theta
        b1_x_cor = b1_x_err * math.cos(bot1_real_theta) - b1_y_err * math.sin(bot1_real_theta)
        b1_y_cor = b1_x_err  * math.sin(bot1_real_theta) + b1_y_err  * math.cos(bot1_real_theta)

        # Finding velocities
        b1_v_x = self.Kp * b1_x_cor
        b1_v_y = self.Kp * b1_y_cor
        b1_w = (self.Kp + 0.5)*b1_theta_err

        values = [b1_v_x, b1_v_y, b1_w]
        # Reshaping it into 3*1 for Inverse Kinematics calculations
        m = np.array(values)
        b1_chassis_vel = m.reshape(3,1)

        
        result1 = np.matmul(matrix, b1_chassis_vel) # Actual Inverse Kinematic multiplication

        # Assigning the wheel velocities
        self.b1_v1 = result1[0][0]
        self.b1_v2 = result1[1][0]
        self.b1_v3 = result1[2][0]

        #Store these velocities into messages



        pass

    def Bot1_CallBack(self, msg):
        self.b1_x = msg.x
        self.b1_y = msg.y
        self.b1_theta = msg.theta

    def Bot2_CallBack(self, msg):
        self.b2_x = msg.x
        self.b2_y = msg.y
        self.b2_theta = msg.theta

    def Bot3_CallBack(self, msg):
        self.b3_x = msg.x
        self.b3_y = msg.y
        self.b3_theta = msg.theta

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
