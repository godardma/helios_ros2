from geometry_msgs.msg import TransformStamped,PoseStamped 
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
import math
import os


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class BoatSimulator(Node):

    def __init__(self):
        super().__init__('boat_simulator')
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1000)
        self.subscription_pose = self.create_subscription(
            Float64,
            'commande',
            self.commande_callback,
            1000)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.mission_finished_serv=self.create_service(Trigger, 'finish', self.finish_callback)

        self.pose = PoseStamped()
        self.pose.header.frame_id="map"


        self.comm=0.
        self.theta,self.x,self.y=0.,0.,0.
        self.moteur=True
        

    def timer_callback(self):
        #voiture de Dubins
        dt=0.02
        v=4
        if self.moteur:
            self.x=self.x+v*np.cos(self.theta)*dt
            self.y=self.y+v*np.sin(self.theta)*dt
            self.theta=self.theta+dt*self.comm

            self.pose.header.stamp = self.get_clock().now().to_msg()
            self.pose.pose.position.x=self.x
            self.pose.pose.position.y=self.y
            q=quaternion_from_euler(0.,0.,self.theta)
            self.pose.pose.orientation.z=q[2]
            self.pose.pose.orientation.w=q[3]
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = 'boat'
            t.transform.translation.x = self.pose.pose.position.x
            t.transform.translation.y =self.pose.pose.position.y
            t.transform.translation.z = self.pose.pose.position.z
            t.transform.rotation.x = self.pose.pose.orientation.x
            t.transform.rotation.y = self.pose.pose.orientation.y
            t.transform.rotation.z = self.pose.pose.orientation.z
            t.transform.rotation.w = self.pose.pose.orientation.w

            self.pose.pose.orientation.z=q[2]
            self.pose.pose.orientation.w=q[3]


            self.tf_broadcaster.sendTransform(t)
            self.pose_publisher.publish(self.pose)


    
    def commande_callback(self,msg):
        # self.get_logger().info('commande :%f'%msg.data)
        if not math.isnan(msg.data):
            self.comm=msg.data

    def finish_callback(self, request, response):
        response.success =True
        response.message ="End Of Mission"
        self.moteur=False

        return response


def main(args=None):
    rclpy.init(args=args)

    boat_simulator = BoatSimulator()

    rclpy.spin(boat_simulator)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
