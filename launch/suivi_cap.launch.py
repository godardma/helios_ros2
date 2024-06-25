from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    # hemisphere_node = Node(
    #     package="hemisphere_v500",
    #     executable="hemisphere_v500_node",

    # )
    gnss_infos_node = Node(
        package="helios_ros2",
        executable="gnss_infos",

    )
    command_node = Node(
        package="helios_ros2",
        executable="command",
        output="screen",
    )
    motors_node = Node(
        package="helios_ros2",
        executable="motors",
        output="screen",
    )
    # ld.add_action(hemisphere_node)
    ld.add_action(gnss_infos_node)
    ld.add_action(command_node)
    ld.add_action(motors_node)
    return ld