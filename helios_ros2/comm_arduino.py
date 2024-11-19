import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_srvs.srv import Trigger

import numpy as np
import math
import serial
import sys
import struct


def parse_command(rot,lin=60):
    return str(rot)+' '+str(lin)+'\n'


class MotorNode(Node):

    def __init__(self):
        super().__init__('command_node')
        self.subscription_pose = self.create_subscription(
            Float64,
            'commande',
            self.commande_callback,
            10)
        self.motors_on=True
        self.mission_finished_serv=self.create_service(Trigger, 'finish', self.finish_callback)
        self.ser = serial.Serial('/dev/narval_motors', 115200, timeout=0.5)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i=0
        
    def timer_callback(self):
        try:
            msg = self.ser.read_until(b'\n').decode('utf-8').strip()
            # self.ser.reset_input_buffer()  # Vide le buffer d'entrée
            # self.ser.reset_output_buffer()  # Vide le buffer de sortie
            # msg_str = str(msg)
            if len(msg)>0:
                self.get_logger().info(msg)
        except:
            pass

    def commande_callback(self,msg):
        try:
            cons_gain=int(msg.data)
            # self.get_logger().info('comm gain :%f'%cons_gain)
            # if cons_gain>119:
            #     cons_gain=119.
            # elif cons_gain<-119:
            #     cons_gain=-119.
            # cons_ard=cons_gain+120
            # self.get_logger().info('comm gain :%f'%cons_gain)
            # self.get_logger().info('data :%f'%msg.data)
            if self.motors_on:
                # self.get_logger().info('commande :%f'%cons_ard)
                # self.ser.write(struct.pack('>B', cons_ard))
                message=parse_command(cons_gain,60)
                self.ser.write(message.encode('utf-8'))
            else:
                self.get_logger().info('mission finie')
                message="0 0\n"
                self.ser.write(message.encode('utf-8'))
                
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
