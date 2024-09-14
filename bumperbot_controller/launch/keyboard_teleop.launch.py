from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    key_teleop = Node(
        package="key_teleop",
        executable="key_teleop",
        output="screen"
    )

    teleop_to_twist = Node(
        package="bumperbot_py_examples",
        executable="teleop_to_stamped"
    )

    return LaunchDescription([
        key_teleop,
        teleop_to_twist
    ])