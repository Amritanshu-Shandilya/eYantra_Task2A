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
# from my_robot_interfaces.srv import NextGoal  
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

DEBUG = False
##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Subscriber
        self.aruco_subscriber = self.create_subscription(Pose2D,'/hb_bot_1/detect_aruco',self.aruco_detect_callback, 10)

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
        self.timer = self.create_timer(0.5, self.inverse_kinematics)

        
        self.Kp = 1

        # client for the "next_goal" service
        # self.cli = self.create_client(NextGoal, 'next_goal')      
        # self.req = NextGoal.Request() 
        self.index = 0


    def aruco_detect_callback(self, msg):
        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta
        print(self.hb_x, self.hb_y, self.hb_theta)

    
    # Method to create a request to the "next_goal" service
    # def send_request(self, request_goal):
    #     self.req.request_goal = request_goal
    #     self.future = self.cli.call_async(self.req)
    #     time.sleep(1)
        

    def inverse_kinematics(self):
        # values = [self.hb_x, self.hb_y, self.hb_theta]

        # For testing purpose we are trying to make a square
        x_goal = [250, 300, 300, 250][self.index]
        y_goal = [200, 200, 250, 250][self.index]
        theta_goal  = [0, math.pi/2, -math.pi, -math.pi/2, 0][self.index]
        self.flag = self.index == 4

        # Finding error nd applying P-controller
        x_err = x_goal - self.hb_x
        y_err = y_goal - self.hb_y
        theta_err = theta_goal - self.hb_theta

        # Frame changing using Rotation matrix
        bot_real_theta = -self.hb_theta
        x_cor = x_err * math.cos(bot_real_theta) - y_err * math.sin(bot_real_theta)
        y_cor = x_err  * math.sin(bot_real_theta) + y_err  * math.cos(bot_real_theta)

        v_x = self.Kp * x_cor
        v_y = self.Kp * y_cor
        w = (self.Kp + 0.5)*theta_err

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

        # print the values, for debugging purpose
        if DEBUG:
            print("Goal:", x_goal, y_goal, theta_goal, self.flag)
            print("Bot Pos:", self.hb_x, self.hb_y, self.hb_theta)
            print("Error:", x_err, y_err, theta_err)
            print("Speed:", v_x, v_y, w)
            print()

        # Apply appropriate force vectors
            #Create the messages and publish the data:
        msg1 = Wrench()
        msg1.force.y = self.v1
        self.v1_publisher.publish(msg1)

        msg2 = Wrench()
        msg2.force.y = self.v2
        self.v2_publisher.publish(msg2)

        msg3 = Wrench()
        msg3.force.y = self .v3
        self.v3_publisher.publish(msg3)

        # this if below let the index increment only if you reach the desired goal
        goal_error = math.sqrt(x_err**2 + y_err**2)
        if goal_error < 0.1 and abs(theta_err) < 0.1:
            self.get_logger().info(f'Reached Goal: x:{x_goal}, y:{y_goal}, theta:{theta_goal}\n')
            self.index += 1
            if self.flag == 1 :
                self.index = 0
    # hb_controller.send_request(hb_controller.index)


def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    hb_controller = HBController()
   
    # Send an initial request with the index from HBController.index
    # hb_controller.send_request(hb_controller.index)
    
    # Main loop
    # while rclpy.ok():

    #     # Check if the service call is done
    #     if hb_controller.future.done():
    #         try:
    #             # response from the service call
    #             response = hb_controller.future.result()
    #         except Exception as e:
    #             hb_controller.get_logger().infselfo(
    #                 'Service call failed %r' % (e,))
    #         else:
    #             #########           GOAL POSE             #########
    #             x_goal      = response.x_goal
    #             y_goal      = response.y_goal
    #             theta_goal  = response.theta_goal
    #             hb_controller.flag = response.end_of_list
    #             ####################################################

                # For testing purpose we are trying to make a square
                # x_goal = [4, -4, -4, 4, 0][HBController.index]
                # y_goal = [4, 4, -4, -4, 0][HBController.index]
                # theta_goal  = [0, math.pi/2, -math.pi, -math.pi/2, 0][HBController.index]
                # HBController.flag = HBController.index == 4
                        
    #             ############     DO NOT MODIFY THIS       #########
    #             hb_controller.index += 1
    #             if hb_controller.flag == 1 :
    #                 hb_controller.index = 0
    #             hb_controller.send_request(hb_controller.index)
    #             ####################################################

        # Spin once to process callbacks
    rclpy.spin(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
