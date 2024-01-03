import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory("jinnode"), "config", "params.yaml"
    )

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    node_motor = Node(
        package="jinnode", executable="motor_node", parameters=[config]
    )
    node_model = Node(
        package="jinnode", executable="model_node")


    ld.add_action(node_microros)
    ld.add_action(node_motor)
    ld.add_action(node_model)

    return ld
