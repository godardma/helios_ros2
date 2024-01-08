from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PointStamped

import rclpy
from rclpy.node import Node

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

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
            PoseStamped,
            'pose',
            self.pose_callback,
            1000)
        self.subscription_target = self.create_subscription(
            Float64,
            'theta_des',
            self.theta_des_callback,
            1000)
        self.comm_publisher = self.create_publisher(Float64, 'commande', 1000)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.theta=0.
        self.theta_sav=0.
        self.theta_des=0.
        
    def timer_callback(self):
        Kp=20.0
        Kd=0.0
        theta_dot=Kp*sawtooth(self.theta_des-self.theta)-Kd*sawtooth(self.theta-self.theta_sav)
        self.theta_sav=self.theta
        command=Float64()
        command.data=theta_dot
        # self.get_logger().info('comm %f'%theta_dot)
        self.comm_publisher.publish(command)
        

    def pose_callback(self,msg):
        roll,pitch,yaw=euler_from_quaternion(msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w)
        self.theta=yaw

    def theta_des_callback(self,msg):
        self.theta_des=msg.data
        






def main(args=None):
    rclpy.init(args=args)

    command_node = CommandNode()

    rclpy.spin(command_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
