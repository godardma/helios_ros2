from std_msgs.msg import Float64
from geometry_msgs.msg import  PointStamped

import rclpy
from rclpy.node import Node

import numpy as np
import math


def sawtooth(x):
    '''
    permet d'avoir une différence d'angle nulle entre theta = 0 et theta = 2*pi
    :param x: différence d'angle
    :return: différence comprise entre [-pi,pi[
    '''
    return (x+np.pi)%(2*np.pi)-np.pi

class CommandNode(Node):

    def __init__(self):
        super().__init__('command_node')
        self.subscription_pose = self.create_subscription(
            PointStamped,
            'obstacle',
            self.point_callback,
            1000)
        self.comm_publisher = self.create_publisher(Float64, 'commande', 1000)

        
        

    def point_callback(self,msg):
        dx,dy=msg.point.x,msg.point.y
        command=Float64()
        command.data=msg.point.x/8.0
        self.comm_publisher.publish(command)
        





def main(args=None):
    rclpy.init(args=args)

    command_node = CommandNode()

    rclpy.spin(command_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
