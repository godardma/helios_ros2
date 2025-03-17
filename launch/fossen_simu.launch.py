from launch import LaunchDescription
from launch_ros.actions import Node

from pyproj import Proj, transform
from pyproj import Transformer

def deg_to_Lamb (x1,y1):
    transformer=Transformer.from_crs(4326,2154,always_xy=True)
    point=[(x1,y1)]
    for pt in transformer.itransform(point):
        return pt
    
ref=[-3.0147,48.1988]
ref_lamb=deg_to_Lamb(ref[0],ref[1])

pathfile_name="triangle_test.txt"

logfile_name="logs_test"


def generate_launch_description():
    ld = LaunchDescription()
    mission_node = Node(
        package="helios_ros2",
        executable="mission_publisher",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
            {"pathfile_name":pathfile_name},
        ]
    )
    boat_node = Node(
        package="usv_simulator",
        executable="boat_simulator.py",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    control_node = Node(
        package="usv_simulator",
        executable="boat_control.py",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    interface_node=Node(
        package="usv_simulator",
        executable="command_to_twist.py",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    navigation_node = Node(
        package="helios_ros2",
        executable="line_follow",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
            {"logfile_name":logfile_name},
        ]
    )
    command_node = Node(
        package="helios_ros2",
        executable="command",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    map_to_ned_node= Node(
        package="helios_ros2",
        executable="map_to_ned",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    ld.add_action(navigation_node)
    ld.add_action(mission_node)
    ld.add_action(boat_node)
    ld.add_action(command_node)
    ld.add_action(control_node)
    ld.add_action(interface_node)
    ld.add_action(map_to_ned_node)
    return ld