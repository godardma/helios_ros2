from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    mission_node = Node(
        package="helios_ros2",
        executable="mission_publisher",
    )
    boat_node = Node(
        package="helios_ros2",
        executable="boat_simulator"
    )
    ld.add_action(mission_node)
    ld.add_action(boat_node)
    return ld