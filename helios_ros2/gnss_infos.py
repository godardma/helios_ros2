from geometry_msgs.msg import TransformStamped,PoseStamped 
from std_msgs.msg import Float64
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from hemisphere_v500.msg import Binary3

import rclpy
from rclpy.node import Node

import numpy as np
import math
import os
from pyproj import Proj, transform
from pyproj import Transformer

def deg_to_Lamb (x1,y1):
    transformer=Transformer.from_crs(4326,2154,always_xy=True)
    point=[(x1,y1)]
    for pt in transformer.itransform(point):
        return pt

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

class GnssInfos(Node):

    def __init__(self):
        super().__init__('gnss_infos')
        self.declare_parameter('X', 253527.41)	#Guerledan
        self.declare_parameter('Y', 6805773.69)
        self.declare_parameter('pathfile_name')
        x_ref= self.get_parameter('X').value
        y_ref=self.get_parameter('Y').value
        self.ref_lamb=[x_ref,y_ref]
        self.get_logger().info('ref %f %f'%(x_ref,y_ref))
        print(x_ref,y_ref)
        self.pose_publisher = self.create_publisher(PoseStamped, 'pose', 1000)
        self.subscription_bin3 = self.create_subscription(
            Binary3,
            '/bin3',
            self.bin3_callback,
            1000)
        self.tf_broadcaster_ned = TransformBroadcaster(self)


        self.tf_broadcaster = TransformBroadcaster(self)

        self.pose = PoseStamped()
        self.pose.header.frame_id="NED"

    
    def bin3_callback(self,msg):
        cap=msg.heading-90. #si heading faire -90
        if cap >180.:
            cap=cap-360.0
        elif cap < -180:
            cap=360+cap
        # self.get_logger().info('cap :%f'%cap)
        q=quaternion_from_euler(0.,0.,cap*np.pi/180.)

        # self.get_logger().info('z :%f'%q[2])
        # self.get_logger().info('w :%f'%q[3])

        lat,lon=msg.latitude,msg.longitude
        X,Y=deg_to_Lamb(lon,lat)
        # self.get_logger().info('X :%f'%X)
        # self.get_logger().info('Y :%f'%Y)

        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.pose.position.y=X-self.ref_lamb[0]
        self.pose.pose.position.x=Y-self.ref_lamb[1]
        self.pose.pose.position.z=0.
        self.pose.pose.orientation.x=0.
        self.pose.pose.orientation.y=0.
        self.pose.pose.orientation.z=q[2]
        self.pose.pose.orientation.w=q[3]

        self.pose_publisher.publish(self.pose)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'NED'
        t.child_frame_id = 'boat'
        t.transform.translation.x = self.pose.pose.position.x
        t.transform.translation.y =self.pose.pose.position.y
        t.transform.translation.z = self.pose.pose.position.z
        t.transform.rotation.x = self.pose.pose.orientation.x
        t.transform.rotation.y = self.pose.pose.orientation.y
        t.transform.rotation.z = self.pose.pose.orientation.z
        t.transform.rotation.w = self.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

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

    gnss_infos = GnssInfos()

    rclpy.spin(gnss_infos)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
