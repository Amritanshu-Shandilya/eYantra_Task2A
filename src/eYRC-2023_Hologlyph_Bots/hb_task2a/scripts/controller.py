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
# Author List:	Anurag,, Amritanshu, Saumitra, Ansh
# Filename:		feedback.py
# Functions:
#			[ Comma separated list of functions in this file ]
# Nodes:		Add your publishing and subscribing node


################### IMPORT MODULES #######################

import rclpy
from rclpy.node import Node
import time
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal  
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D 


# NOTE: You are strictly NOT-ALLOWED to use "cmd_vel" or "odom" topics in this task
# from geometry_msgs.msg import Twist
#from nav_msgs.msg import Odometry
           
##############################################################


# Initialize Global variables


################# ADD UTILITY FUNCTIONS HERE #################

##############################################################


# Define the HBController class, which is a ROS node
class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')
        
        # Initialze Subscriber
        self.aruco_subscriber = self.create_subscription(Pose2D,'/detect_aruco',self.aruco_detect_callback, 1)

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

        # For maintaining control loop rate.
        self.rate = self.create_rate(100)

        
        self.Kp = 1

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0


    def aruco_detect_callback():
        pass
    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        time.sleep(1)
        

    def inverse_kinematics():
        ############ ADD YOUR CODE HERE ############

        # INSTRUCTIONS & HELP : 
        #	-> Use the target velocity you calculated for the robot in previous task, and
        #	Process it further to find what proportions of that effort should be given to 3 individuals wheels !!
        #	Publish the calculated efforts to actuate robot by applying force vectors on provided topics
        ############################################
        pass


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
                
                # <NEED TO VERIFY THIS>
                # Calculate Error from feedback
                error_x = x_goal - HBController.hb_x
                error_y = y_goal - HBController.hb_y
                error_theta = theta_goal - HBController.hb_theta

                bot_real_theeta = -HBController.hb_theta         # the theeta in Odometry is inverse in gobal scope

                    # Finally use the error and orientation of the bot and calculate the x and y velocity
                    # using coordinate transformation and apply the k controller rate
                v_x = HBController.Kp * (error_x * math.cos(bot_real_theeta) - error_y * math.sin(bot_real_theeta))
                v_y = HBController.Kp * (error_x * math.sin(bot_real_theeta) + error_y * math.cos(bot_real_theeta))

                    # extra 0.5 k controller rate for faster angular movement
                w = (HBController.Kp + 0.5) * error_theta 



                # Change the frame by using Rotation Matrix (If you find it required)

                # Calculate the required velocity of bot for the next iteration(s)
                
                # Find the required force vectors for individual wheels from it.(Inverse Kinematics)

                # Apply appropriate force vectors

                # Modify the condition to Switch to Next goal (given position in pixels instead of meters)
                        
                ############     DO NOT MODIFY THIS       #########
                hb_controller.index += 1
                if hb_controller.flag == 1 :
                    hb_controller.index = 0
                hb_controller.send_request(hb_controller.index)
                ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(hb_controller)
    
    # Destroy the node and shut down ROS
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
