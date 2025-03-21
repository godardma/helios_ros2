import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from std_srvs.srv import Trigger

import numpy as np
import math
import serial
import sys
import struct


def parse_command(rot,lin,sum):
    return str(rot)+' '+str(lin)+' '+str(sum)+'\n'


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
        self.voltage_publisher = self.create_publisher(Float64, 'voltage', 10)
        self.current_publisher = self.create_publisher(Float64, 'current', 10)
        self.motor_battery_publisher = self.create_publisher(Float64, 'motor_battery', 10)
        self.elec_battery_publisher = self.create_publisher(Float64, 'elec_battery', 10)
        self.ser = serial.Serial('/dev/narval_motors', 115200, timeout=0.5)
        timer_period = 0.002  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i=0
        self.cons_gain=0.
        message="0 0\n"
        self.ser.write(message.encode('utf-8'))
        
    def timer_callback(self):
        try:
            msg = self.ser.read_until(b'\n').decode('utf-8').strip()
            # self.ser.reset_input_buffer()  # Vide le buffer d'entrée
            # self.ser.reset_output_buffer()  # Vide le buffer de sortie
            # msg_str = str(msg)
            if len(msg)>0:
            	if "batteries" in msg:
                    splitted=msg.split(" ")
                    motor,elec,voltage,current=Float64(),Float64(),Float64(),Float64()
                    motor.data=float(splitted[1])
                    elec.data=float(splitted[2])
                    voltage.data=float(splitted[3])
                    current.data=float(splitted[4])

                    self.motor_battery_publisher.publish(motor)
                    self.elec_battery_publisher.publish(elec)
                    self.voltage_publisher.publish(voltage)
                    self.current_publisher.publish(current)
        except Exception as e:
            self.get_logger().info(e)

    def commande_callback(self,msg):
        try:
            self.cons_gain=int(msg.data)
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
                message=parse_command(self.cons_gain,50, 50+self.cons_gain)
                # self.get_logger().info(message)
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
