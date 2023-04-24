from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32,TransformStamped,PointStamped 
from tf2_ros import TransformBroadcaster

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


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        ref=[-3.0147,48.1988] #lon,lat
        self.ref_lamb=deg_to_Lamb(ref[0],ref[1])
        print(self.ref_lamb)
        self.path_publisher = self.create_publisher(PointCloud, 'path', 1000)
        self.target_publisher = self.create_publisher(PointStamped, 'target', 1000)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        f=open("../workspaceRos2/src/helios_ros2/path/triangle_test.txt")
        lines=f.readlines()
        self.path = PointCloud()
        self.path.header.frame_id="map"
        for line in lines:
            tab=line.split(",")
            lat,lon=float(tab[0]),float(tab[1])
            X,Y=deg_to_Lamb(lon,lat)
            pt=Point32()
            pt.x,pt.y=X-self.ref_lamb[0],Y-self.ref_lamb[1]
            self.path.points.append(pt)

        self.path_publisher.publish(self.path)

    def timer_callback(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'frame'
        t.transform.translation.x = 0.0
        t.transform.translation.y =0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)
        self.path_publisher.publish(self.path)
        pt=PointStamped()
        pt.point.x=self.path.points[0].x
        pt.point.y=self.path.points[0].y
        pt.header.frame_id="map"
        self.target_publisher.publish(pt)



def main(args=None):
    rclpy.init(args=args)

    minimal_service = MissionManager()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
