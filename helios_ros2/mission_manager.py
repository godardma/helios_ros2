from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32,TransformStamped,PointStamped 
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

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


class MissionManager(Node):

    def __init__(self):
        super().__init__('mission_manager')
        self.declare_parameter('X', 0.)
        self.declare_parameter('Y', 0.)
        self.declare_parameter('pathfile_name')
        x_ref= self.get_parameter('X').value
        y_ref=self.get_parameter('Y').value
        self.ref_lamb=[x_ref,y_ref]
        self.path_publisher = self.create_publisher(PointCloud, 'path', 10)
        self.path_done_publisher = self.create_publisher(PointCloud, 'path_done', 10)
        self.target_publisher = self.create_publisher(PointStamped, 'target', 1)
        self.trigger_serv=self.create_service(Trigger, 'trigger', self.trigger_callback)
        self.cli = self.create_client(Trigger, 'finish')
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.req=Trigger.Request()

        pathfile_name=self.get_parameter('pathfile_name').value
        self.get_logger().info('file %s loaded'%pathfile_name)
        

        package_share_directory = get_package_share_directory('helios_ros2')

        f=open(package_share_directory+"/path/"+pathfile_name)
        lines=f.readlines()
        self.path = PointCloud()
        self.path_done = PointCloud()
        self.path.header.frame_id="NED"
        self.path_done.header.frame_id="NED"

        if pathfile_name[-3:]=="txt":
            self.get_logger().info('txt format')
            for line in lines:
                tab=line.split(",")
                lon,lat=float(tab[0]),float(tab[1])
                # self.get_logger().info('lon %f'%lon)
                # self.get_logger().info('lat %f'%lat)
                X,Y=deg_to_Lamb(lon,lat)
                pt=Point32()
                pt.y,pt.x=X-self.ref_lamb[0],Y-self.ref_lamb[1]
                self.path.points.append(pt)
        elif pathfile_name[-3:]=="csv":
            self.get_logger().info('csv format')
            for line in lines[1:]:
                tab=line.split(",")
                lon,lat=float(tab[2]),float(tab[1])
                # self.get_logger().info('lon %f'%lon)
                # self.get_logger().info('lat %f'%lat)
                X,Y=deg_to_Lamb(lon,lat)
                pt=Point32()
                pt.y,pt.x=X-self.ref_lamb[0],Y-self.ref_lamb[1]
                self.path.points.append(pt)
        

        self.path_publisher.publish(self.path)
        self.target=PointStamped()
        self.target.point.x=self.path.points[0].x
        self.target.point.y=self.path.points[0].y
        self.target.header.frame_id="NED"
        self.target_publisher.publish(self.target)
        

    def timer_callback(self):
        self.path_publisher.publish(self.path)
        self.path_done_publisher.publish(self.path_done)
        # self.target_publisher.publish(self.target)


    def trigger_callback(self, request, response):
        response.success =True
        if len(self.path.points)>1:
            old_target=self.path.points[0]
            self.path.points=self.path.points[1:]
            self.path_done.points.append(old_target)
            self.get_logger().info('Changing target')
            response.message ="Next point"
            self.target.point.x=self.path.points[0].x
            self.target.point.y=self.path.points[0].y
            self.target.header.frame_id="NED"
            self.target_publisher.publish(self.target)
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
