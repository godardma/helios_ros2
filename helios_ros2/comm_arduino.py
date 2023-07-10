import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_srvs.srv import Trigger

import numpy as np
import math
import serial
import sys
import struct

class MotorNode(Node):

    def __init__(self):
        super().__init__('command_node')
        self.subscription_pose = self.create_subscription(
            Float64,
            'commande',
            self.commande_callback,
            1000)
        self.motors_on=True
        self.mission_finished_serv=self.create_service(Trigger, 'finish', self.finish_callback)
        self.ser = serial.Serial('/dev/narval_motors', 115200, timeout=0.5)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i=0
        
    def timer_callback(self):
        msg = self.ser.readline()
        msg_str = str(msg)
        if  len(msg_str)>6:
            self.i+=1
            if self.i%10==0:
                self.get_logger().info(msg_str[2:-5])

    def commande_callback(self,msg):
        try:
            cons_gain=int(msg.data*100)
            if cons_gain>120:
                cons_gain=120.
            elif cons_gain<-120:
                cons_gain=-120.
            cons_ard=cons_gain+120.
            if self.motors_on:
                # self.get_logger().info('commande :%f'%cons_gain)
                self.ser.write(struct.pack('>B', cons_gain))
            else:
                self.get_logger().info('mission finie')
                self.ser.write(struct.pack('>B', 248))
                
        except:
            pass

    def finish_callback(self, request, response):
        self.motors_on=False
        response.success =True
        response.message ="End Of Mission"
        self.moteur=False
        self.get_logger().info('fin de mission')

        return response






def main(args=None):
    rclpy.init(args=args)

    motor_node = MotorNode()

    rclpy.spin(motor_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
