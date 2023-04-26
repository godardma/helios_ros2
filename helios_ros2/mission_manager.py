from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32,TransformStamped,PointStamped 
from std_srvs.srv import Trigger
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
        self.declare_parameter('X', 0.)
        self.declare_parameter('Y', 0.)
        self.declare_parameter('pathfile_name')
        x_ref= self.get_parameter('X').value
        y_ref=self.get_parameter('Y').value
        self.ref_lamb=[x_ref,y_ref]
        self.path_publisher = self.create_publisher(PointCloud, 'path', 1000)
        self.path_done_publisher = self.create_publisher(PointCloud, 'path_done', 1000)
        self.target_publisher = self.create_publisher(PointStamped, 'target', 1000)
        self.trigger_serv=self.create_service(Trigger, 'trigger', self.trigger_callback)
        self.cli = self.create_client(Trigger, 'finish')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.req=Trigger.Request()

        pathfile_name=self.get_parameter('pathfile_name').value
        self.get_logger().info('file %s loaded'%pathfile_name)
        f=open("src/helios_ros2/path/"+pathfile_name+".txt")
        lines=f.readlines()
        self.path = PointCloud()
        self.path_done = PointCloud()
        self.path.header.frame_id="map"
        self.path_done.header.frame_id="map"
        for line in lines:
            tab=line.split(",")
            lon,lat=float(tab[0]),float(tab[1])
            X,Y=deg_to_Lamb(lon,lat)
            pt=Point32()
            pt.x,pt.y=X-self.ref_lamb[0],Y-self.ref_lamb[1]
            self.path.points.append(pt)

        self.path_publisher.publish(self.path)
        pt=PointStamped()
        pt.point.x=self.path.points[0].x
        pt.point.y=self.path.points[0].y
        pt.header.frame_id="map"
        self.target_publisher.publish(pt)
        

    def timer_callback(self):
        self.path_publisher.publish(self.path)
        self.path_done_publisher.publish(self.path_done)
        

    def trigger_callback(self, request, response):
        response.success =True
        if len(self.path.points)>1:
            old_target=self.path.points[0]
            self.path.points=self.path.points[1:]
            self.path_done.points.append(old_target)
            self.get_logger().info('Changing target')
            response.message ="Next point"
            pt=PointStamped()
            pt.point.x=self.path.points[0].x
            pt.point.y=self.path.points[0].y
            pt.header.frame_id="map"
            self.target_publisher.publish(pt)
        else:
            self.future = self.cli.call_async(self.req)
            self.get_logger().info('Mission finished')
            response.message ="Finish"


        return response


def main(args=None):
    rclpy.init(args=args)

    mission_manager = MissionManager()

    rclpy.spin(mission_manager)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
