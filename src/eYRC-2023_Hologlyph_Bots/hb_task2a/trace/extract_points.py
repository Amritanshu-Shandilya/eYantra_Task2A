import rclpy
from rclpy.node import Node
import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal  
from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Pose2D 


class HBController(Node):
    def __init__(self):
        super().__init__('hb_controller')

        self.cli = self.create_client(NextGoal, 'next_goal')      
        self.req = NextGoal.Request() 
        self.index = 0
        self.flag = 0

    def send_request(self, request_goal):
        self.req.request_goal = request_goal
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    hb_controller = HBController()
    hb_controller.send_request(hb_controller.index)
    while rclpy.ok():
        if hb_controller.future.done():
            try:
                response = hb_controller.future.result()
            except Exception as e:
                hb_controller.get_logger().infselfo(
                    'Service call failed %r' % (e,))
            else:
                x_goal      = response.x_goal
                y_goal      = response.y_goal
                theta_goal  = response.theta_goal
                hb_controller.flag = response.end_of_list
                print(x_goal, y_goal)
                if hb_controller.flag:
                    break

                hb_controller.index += 1
                hb_controller.send_request(hb_controller.index)
        rclpy.spin_once(hb_controller)
    
    hb_controller.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()
