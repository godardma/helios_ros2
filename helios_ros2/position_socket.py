import rclpy
from rclpy.node import Node
from rclpy.time import Time
from hemisphere_v500.msg import StampedString
import math

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from tcpsocket import TcpSocket

class PosSocketNode(Node):
    """ """
    def __init__(self):
        """ """
        super().__init__('position_socket_server')
        self.nmeaSub = self.create_subscription(StampedString,'nmea',self.nmeaCbk,1)

        #
        self.server = None
        self.initServer('localhost',20000)
        # self.initServer('10.43.20.225',20000)
        #

    def nmeaCbk(self,msg):
        self.server.keep()
        
        if self.server.isConnected():

            s = msg.data
            if s[:3]=="$GP":
                # self.get_logger().info(msg.data)
                self.server.send(bytes(s+"\r\n",'ascii'))


    def initServer(self,ip,tcpPort):
        self.server = TcpSocket(ip,tcpPort,server = True)
        #self.server = TcpSocket(ip,tcpPort,server = False)


def main():
    rclpy.init(args = None)
    posSocketNode = PosSocketNode()
    rclpy.spin(posSocketNode)
    posSocketNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

