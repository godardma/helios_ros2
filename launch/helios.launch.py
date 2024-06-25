from launch import LaunchDescription
from launch_ros.actions import Node

from pyproj import Proj, transform
from pyproj import Transformer

def deg_to_Lamb (x1,y1):
    transformer=Transformer.from_crs(4326,2154,always_xy=True)
    point=[(x1,y1)]
    for pt in transformer.itransform(point):
        return pt
    
ref=[-3.0147,48.1988]     #Guerledan
# ref=[-4.4738,48.4183]   #Brest
ref_lamb=deg_to_Lamb(ref[0],ref[1])

pathfile_name="test_guerl"

logfile_name="logs_der"


def generate_launch_description():
    ld = LaunchDescription()
    hemisphere_node = Node(
        package="hemisphere_v500",
        executable="hemisphere_v500_node",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
            {"pathfile_name":pathfile_name},
        ]
    )
    gnss_infos_node = Node(
        package="helios_ros2",
        executable="gnss_infos",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
            {"pathfile_name":pathfile_name},
        ]
    )
    mission_node = Node(
        package="helios_ros2",
        executable="mission_publisher",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
            {"pathfile_name":pathfile_name},
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
    motors_node = Node(
        package="helios_ros2",
        executable="motors",
        output="screen",
        parameters=[
            {"X": ref_lamb[0]},
            {"Y": ref_lamb[1]},
        ]
    )
    # ld.add_action(hemisphere_node)
    ld.add_action(gnss_infos_node)
    ld.add_action(navigation_node)
    ld.add_action(mission_node)
    ld.add_action(command_node)
    ld.add_action(motors_node)
    return ld