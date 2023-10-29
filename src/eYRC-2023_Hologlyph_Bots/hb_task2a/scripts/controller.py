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


# Initialize Global variables
values = [-0.33, 0.58, 0.33, -0.33, -0.58, 0.33, 0.67, 0, 0.33]
    #Create a 3x3 numpy matrix out of this list
matrix = np.array(values)
matrix = matrix.reshape(3,3)


################# ADD UTILITY FUNCTIONS HERE #################

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

        
        self.Kp = 1

        # client for the "next_goal" service
        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0


    def aruco_detect_callback(self, msg):
        self.hb_x = msg.x
        self.hb_y = msg.y
        self.hb_theta = msg.theta
        print(self.hb_x, self.hb_y, self.hb_theta)

    
    # Method to create a request to the "next_goal" service
    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)
        time.sleep(1)
        

    def inverse_kinematics(self):
        values = [self.hb_x, self.hb_y, self.hb_theta]
        m2 = np.array(values)
        coordinates = m2.reshape(3,1)

        # multiply these 2 matrices to find the wheel velocities
        result = np.matmul(matrix, coordinates)
        
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

                #Create the messages and publish the data:
                msg1 = Wrench()
                msg1.force.y = HBController.v1
                HBController.v1_publisher.publish(msg1)

                msg2 = Wrench()
                msg2.force.y = HBController.v2
                HBController.v2_publisher.publish(msg2)

                msg3 = Wrench()
                msg3.force.y = HBController.v3
                HBController.v3_publisher.publish(msg3)
                        
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
