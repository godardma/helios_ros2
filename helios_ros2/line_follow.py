from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

import matplotlib.pyplot as plt
import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

def deg_to_Lamb (x1,y1):
    transformer=Transformer.from_crs(4326,2154,always_xy=True)
    point=[(x1,y1)]
    for pt in transformer.itransform(point):
        return pt

def sawtooth(x):
    '''
    permet d'avoir une différence d'angle nulle entre theta = 0 et theta = 2*pi
    :param x: différence d'angle
    :return: différence comprise entre [-pi,pi[
    '''
    return (x+np.pi)%(2*np.pi)-np.pi

def dist(xrob,yrob,xaim,yaim):
    return ((xrob-xaim)**2+(yrob-yaim)**2)**0.5

def guidage(a,b,m,r=7):
    '''
    fonction permettant de suivre la ligne souhaitée
    :param a: point de départ de la ligne
    :param b: point d'arrivée
    :param m: position du bateau
    :param r: precision (on a pris 7m par défaut)
    :return: angle désiré
    '''
    u = (b-a)/(np.linalg.norm(b-a))
    e = np.linalg.det(np.hstack((u,m-a)))
    bx,by=b.flatten()
    ax,ay=a.flatten()
    phi = np.arctan2(by-ay,bx-ax)
    theta_barre = phi-np.arctan2(e,r)

    return theta_barre

def validation(a,b,m):
    '''
    fonction permettant de suivre la ligne souhaitée
    :param a: point de départ de la ligne
    :param b: point d'arrivée
    :param m: position du bateau
    :param r: precision (on a pris 7m par défaut)
    :return: angle désiré
    '''
    iss=np.dot((b-a).T,(b-m))<0.
    return iss


class LineFollow(Node):

    def __init__(self):
        super().__init__('line_follow')
        ref=[-3.0147,48.1988] #lon,lat
        self.ref_lamb=deg_to_Lamb(ref[0],ref[1])
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            'pose',
            self.pose_callback,
            1000)
        self.subscription_target = self.create_subscription(
            PointStamped,
            'target',
            self.target_callback,
            1000)
        self.thetad_publisher = self.create_publisher(Float64, 'theta_des', 1000)
        self.cli = self.create_client(Trigger, 'trigger')
        self.req=Trigger.Request()
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.a,self.b=np.array([[0],[0]]),np.array([[0],[0]])
        self.m=np.array([[0],[0]])
        

    def pose_callback(self,msg):
        self.m[0,0]=msg.pose.position.x
        self.m[1,0]=msg.pose.position.y

    def target_callback(self,msg):
        self.a[0,0]=self.b[0,0]
        self.a[1,0]=self.b[1,0]
        self.b[0,0]=msg.point.x
        self.b[1,0]=msg.point.y

    def timer_callback(self):
        if(validation(self.a,self.b,self.m)):
            self.future = self.cli.call_async(self.req)
            self.get_logger().info('%s'%self.future)
        theta_des=guidage(self.a,self.b,self.m,1)
        msg_td=Float64()
        msg_td.data=theta_des
        # self.get_logger().info('theta : %f'%theta_des)
        self.thetad_publisher.publish(msg_td)




def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
