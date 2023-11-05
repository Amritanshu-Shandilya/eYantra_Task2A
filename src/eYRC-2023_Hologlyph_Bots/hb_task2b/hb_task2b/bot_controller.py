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
        self.bot1_v1_publisher = self.create_publisher(Wrench,'/hb_bot_1/left_wheel_force',10)
        self.bot1_v2_publisher = self.create_publisher(Wrench,'/hb_bot_1/right_wheel_force',10)
        self.bot1_v3_publisher =self.create_publisher(Wrench,'/hb_bot_1/rear_wheel_force',10)

        # Bot 2 publishers:
        self.bot2_v1_publisher = self.create_publisher(Wrench,'/hb_bot_2/left_wheel_force',10)
        self.bot2_v2_publisher = self.create_publisher(Wrench,'/hb_bot_2/right_wheel_force',10)
        self.bot2_v3_publisher =self.create_publisher(Wrench,'/hb_bot_2/rear_wheel_force',10)

        # Bot 3 publishers:  
        self.bot3_v1_publisher = self.create_publisher(Wrench,'/hb_bot_3/left_wheel_force',10)
        self.bot3_v2_publisher = self.create_publisher(Wrench,'/hb_bot_3/right_wheel_force',10)
        self.bot3_v3_publisher =self.create_publisher(Wrench,'/hb_bot_3/rear_wheel_force',10)

	    # Initialise the required variables
        # FOR POSITIONS
        # Bot 1:
        self.b1_x = []
        self.b1_y = []
        self.b1_theta = 0.0

        # Bot 2:
        self.b2_x = []
        self.b2_y = []
        self.b2_theta = 0.0

        # Bot 3:
        self.b3_x = []
        self.b3_y = []
        self.b3_theta = 0.0

        #FOR VELOCITIES
        # Bot 1:
        self.b1_v1 = 0.
        self.b1_v2 = 0.
        self.b1_v3 = 0.

        # Bot 2:
        self.b2_v1 = 0.
        self.b2_v2 = 0.
        self.b2_v3 = 0.
        
        # Bot 3:
        self.b3_v1 = 0.
        self.b3_v2 = 0.
        self.b3_v3 = 0.


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
        self.timer = self.create_timer(0.5, self.inverse_kinematics)


        self.Kp1 = 4
        self.index1 = 0
        self.flag1 = 0

    def inverse_kinematics(self):
        ############ DEMO GOALS ############
        b1_x_goal = [175, 125, 125, 175][self.index1]
        b1_y_goal = [50, 50, 100, 100][self.index1]
        b1_theta_goal = [0, math.pi/2, -math.pi, -math.pi/2, 0][self.index1]
        self.flag1 = self.index1 == 4

        ####################################

        #Finding errors and applying p controllers
        b1_x_err = b1_x_goal - self.b1_x
        b1_y_err = b1_y_goal - self.b1_y
        b1_theta_err = b1_theta_goal - self.b1_theta
        
        # Changing the frame using rotation matrix
        b1_angle = -self.b1_theta
        b1_x_cor = b1_x_err * math.cos(b1_angle) - b1_y_err * math.sin(b1_angle)
        b1_y_cor = b1_x_err  * math.sin(b1_angle) + b1_y_err  * math.cos(b1_angle)

        # P-controllers:
        b1_vX = self.Kp1*b1_x_cor
        b1_vY = self.Kp1*b1_y_cor
        b1_w = (self.Kp1+0.5)*b1_theta_err

        # Inverse Kinematics Part
        m = np.array([b1_vX, b1_vY, b1_w])
        b1_chassis_vel = m.reshape(3,1)
        result = np.matmul(matrix, b1_chassis_vel)
        
        # Storing the velocities in the bot
        self.b1_v1 = result[0][0]
        self.b1_v2 = result[1][0]
        self.b1_v3 = result[2][0]

        # Creating messages
        b1_msg1 = Wrench()
        b1_msg1.force.y = self.b1_v1
        self.bot1_v1_publisher.publish(b1_msg1)

        b1_msg2 = Wrench()
        b1_msg2.force.y = self.b1_v2
        self.bot1_v1_publisher.publish(b1_msg2)

        b1_msg3 = Wrench()
        b1_msg3.force.y = self.b1_v3
        self.bot1_v1_publisher.publish(b1_msg3)

        # this if below let the index increment only if you reach the desired goal
        goal_error = math.sqrt(b1_x_err**2 + b1_y_err**2)
        if goal_error < 0.1 and abs(b1_theta_err) < 2:
            self.get_logger().info(f'Reached Goal: x:{b1_x_goal}, y:{b1_y_goal}, theta:{b1_theta_goal}\n')
            self.index1 += 1
            if self.flag1 == 1 :
                self.index1 = 0


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
