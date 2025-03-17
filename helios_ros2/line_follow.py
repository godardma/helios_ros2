from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, PointStamped
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

import numpy as np
import math
from pyproj import Proj, transform
from pyproj import Transformer

from ament_index_python.packages import get_package_share_directory


def deg_to_Lamb (x1,y1):
    transformer=Transformer.from_crs(4326,2154,always_xy=True)
    point=[(x1,y1)]
    for pt in transformer.itransform(point):
        return pt
    
def Lamb_to_deg (x1,y1):
    transformer=Transformer.from_crs(2154,4326,always_xy=True)
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
    try :
        u = (b-a)/(np.linalg.norm(b-a))
        e = np.linalg.det(np.hstack((u,m-a)))
        bx,by=b.flatten()
        ax,ay=a.flatten()
        phi = np.arctan2(by-ay,bx-ax)
        theta_barre = phi-np.arctan2(e,r)

        return theta_barre
    except Exception as e:
        return 0

    # return theta_barre

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
        self.iter=0
        self.subscription_pose = self.create_subscription(
            PoseStamped,
            'pose',
            self.pose_callback,
            1)
        self.subscription_target = self.create_subscription(
            PointStamped,
            'target',
            self.target_callback,
            1)
        self.thetad_publisher = self.create_publisher(Float64, 'theta_des', 1000)
        self.declare_parameter('logfile_name')
        self.declare_parameter('X', 0.)
        self.declare_parameter('Y', 0.)
        self.x_ref= self.get_parameter('X').value
        self.y_ref=self.get_parameter('Y').value
        logfile_name=self.get_parameter('logfile_name').value
        self.get_logger().info('logs written in %s'%logfile_name)

        package_share_directory = get_package_share_directory('helios_ros2')
        self.f_traj=open(package_share_directory+"/../../../../src/helios_ros2/logs/"+logfile_name+"_traj.txt","w")
        self.f_path=open(package_share_directory+"/../../../../src/helios_ros2/logs/"+logfile_name+"_path.txt","w")
        self.cli = self.create_client(Trigger, 'trigger')
        self.req=Trigger.Request()
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.a,self.b=np.array([[0],[0]]),np.array([[0],[0]])
        self.m=np.array([[0],[0]])
        

    def pose_callback(self,msg):
        try:
            self.m[0,0]=msg.pose.position.x
            self.m[1,0]=msg.pose.position.y
            x,y=msg.pose.position.y+self.x_ref,msg.pose.position.x+self.y_ref
            pt=Lamb_to_deg(x,y)
            lon,lat=pt[0],pt[1]
            self.f_traj.write(str(lon)+","+str(lat)+"\n")
        except Exception as e:
            self.get_logger().info('Exception caught')
            pass

    def target_callback(self,msg):
        # if self.a.all()!=self.b.all():
        self.get_logger().info('changing')
        self.a[0,0]=self.b[0,0]
        self.a[1,0]=self.b[1,0]
        self.b[0,0]=msg.point.x
        self.b[1,0]=msg.point.y
        x,y=msg.point.y+self.x_ref,msg.point.x+self.y_ref
        pt=Lamb_to_deg(x,y)
        lon,lat=pt[0],pt[1]
        self.f_path.write(str(lon)+","+str(lat)+"\n")

    def timer_callback(self):
        # val=validation(self.a,self.b,self.m)
        # self.get_logger().info('val : %f'%val)

        # if self.iter>0:
        #     self.iter+=1
        if(validation(self.a,self.b,self.m)):
            self.iter+=1
            if self.iter==2:
                self.future = self.cli.call_async(self.req)
                self.iter=0
        
            # if self.iter==0:
            #     self.iter+=1
            # elif self.iter==5:
            #     self.future = self.cli.call_async(self.req)
        else:
            self.iter=0
        theta_des=guidage(self.a,self.b,self.m,7)
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
