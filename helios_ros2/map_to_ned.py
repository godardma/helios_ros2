from geometry_msgs.msg import TransformStamped,PoseStamped 
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

import rclpy
from rclpy.node import Node

import numpy as np
import math
import os


class TfToNed(Node):

    def __init__(self):
        super().__init__('td_to_ned')
        self.tf_broadcaster_ned = TransformBroadcaster(self)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        

    def timer_callback(self):

            t_ned = TransformStamped()
            t_ned.header.stamp = self.get_clock().now().to_msg()
            t_ned.header.frame_id = 'map'
            t_ned.child_frame_id = 'NED'
            t_ned.transform.translation.x = 0.
            t_ned.transform.translation.y =0.
            t_ned.transform.translation.z = 0.
            t_ned.transform.rotation.x = 0.
            t_ned.transform.rotation.y = -1.
            t_ned.transform.rotation.z =0.
            t_ned.transform.rotation.w = 0.

            self.tf_broadcaster_ned.sendTransform(t_ned)


    

def main(args=None):
    rclpy.init(args=args)

    td_to_ned = TfToNed()

    rclpy.spin(td_to_ned)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
