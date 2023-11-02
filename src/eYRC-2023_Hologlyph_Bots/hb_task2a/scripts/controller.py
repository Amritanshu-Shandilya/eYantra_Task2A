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


# Team ID:		2883
# Author List:	Anurag, Amritanshu, Saumitra, Ansh
# Filename:		feedback.py
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
from my_robot_interfaces.srv import NextGoal  
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D 


# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
# from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
           
##############################################################


################# ADD UTILITY FUNCTIONS HERE #################


# Initialize Global variables
values = [-0.33, 0.58, 0.33, -0.33, -0.58, 0.33, 0.67, 0, 0.33]
    #Create a 3x3 numpy matrix out of this list
matrix = np.array(values)
matrix = matrix.reshape(3,3)

DEBUG = True
##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Subscriber
        self.aruco_subscriber = self.create_subscription(Pose2D,'/detect_aruco',self.aruco_detect_callback, 10)

	    #	Use the below given topics to generate motion for the robot.
        #   /hb_bot_1/left_wheel_force,
	    #   /hb_bot_1/right_wheel_force,
	    #   /hb_bot_1/rear_wheel_force


        #Left_wheel
        self.v1_publisher = self.create_publisher(Wrench,'/hb_bot_1/left_wheel_force',1)
        #Right_wheel
        self.v2_publisher = self.create_publisher(Wrench,'/hb_bot_1/right_wheel_force',1)
        #Rear_wheel
        self.v3_publisher = self.create_publisher(Wrench,'/hb_bot_1/rear_wheel_force',1)


        # Variable to hold bot's current position
        self.hb_x = 0.
        self.hb_y = 0.
        self.hb_theta = 0.

        #Variables to hold wheel velocities
        self.v1 = 0.
        self.v2 = 0.
        self.v3 = 0.

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        
        self.Kp = 2

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        self.flag = 0


    def aruco_detect_callback(self, msg):
        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta

    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        


    def inverse_kinematics(self, v_x, v_y, w):
        # Passing this values in as a 3*1 matrix for multiplication with the matrix declated at the top
        values = [v_x, v_y, w]
        m2 = np.array(values)
        chassis_vel = m2.reshape(3,1)

        # multiply these 2 matrices to find the wheel velocities
        result = np.matmul(matrix, chassis_vel)
        
        #storing these velocities in the global variables
        self.v1 = result[0][0]
        self.v2 = result[1][0]
        self.v3 = result[2][0]


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from HBController.index
    hb_controller.send_request(hb_controller.index)
    
    # Main loop
    while rclpy.ok():

        # Check if the service call is done
        if hb_controller.future.done():
            try:
                # response from the service call
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                #########           GOAL POSE             #########
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                ####################################################

                # x_goal = [100, 400, 400, 100, 250][hb_controller.index]
                # y_goal = [100, 100, 400, 400, 250][hb_controller.index]
                # theta_goal  = [0, 0, 0, 0, 0][hb_controller.index]
                # hb_controller.flag = hb_controller.index == 4

                x_goal = 250 + x_goal
                y_goal = 250 - y_goal

                # Finding error nd applying P-controller
                x_err = x_goal - hb_controller.hb_x
                y_err = y_goal - hb_controller.hb_y
                theta_err = theta_goal - hb_controller.hb_theta

                # Frame changing using Rotation matrix
                bot_real_theta = -hb_controller.hb_theta
                x_cor = x_err * math.cos(bot_real_theta) - y_err * math.sin(bot_real_theta)
                y_cor = x_err  * math.sin(bot_real_theta) + y_err  * math.cos(bot_real_theta)

                v_x = hb_controller.Kp * x_cor
                v_y = hb_controller.Kp * y_cor
                w = (hb_controller.Kp + 0.5)*theta_err

                # print the values, for debugging purpose
                if DEBUG:
                    print("Goal:", x_goal, y_goal, theta_goal, hb_controller.flag)
                    print("Bot Pos:", hb_controller.hb_x, hb_controller.hb_y, hb_controller.hb_theta)
                    print("Error:", x_err, y_err, theta_err)
                    print("Speed:", v_x, v_y, w)
                    print()

                hb_controller.inverse_kinematics(v_x, v_y, w)

                # Apply appropriate force vectors
                #Create the messages and publish the data:
                msg1, msg2, msg3 = Wrench(), Wrench(), Wrench()
                msg1.force.y, msg2.force.y, msg2.force.y = hb_controller.v1, hb_controller.v2, hb_controller.v2

                hb_controller.v1_publisher.publish(msg1)
                hb_controller.v2_publisher.publish(msg2)
                hb_controller.v3_publisher.publish(msg3)

                goal_error = math.sqrt(x_err**2 + y_err**2)
                if goal_error < 5:
                    hb_controller.get_logger().info(f'Reached Goal: x:{x_goal}, y:{y_goal}, theta:{theta_goal}\n')
                    ############     DO NOT MODIFY THIS       #########
                    hb_controller.index += 1
                    if hb_controller.flag == 1 :
                        hb_controller.index = 0
                    hb_controller.send_request(hb_controller.index)
                    ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
        time.sleep(0.1)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
