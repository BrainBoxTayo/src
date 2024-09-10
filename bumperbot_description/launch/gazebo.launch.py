from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from os import pathsep
from ament_index_python import get_package_share_directory, get_package_prefix
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    bumperbot_description = get_package_share_directory(
        "bumperbot_description")
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")

    model_path = os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(get_package_share_directory(
            "bumperbot_description"), "urdf", "bumperbot.urdf.xacro"),
        description="Absolute path to robot urdf file"
    )
    # convert from xacro to urdf using the Command function
    # the Launch Configuration is looking for the model argument
    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(""))))
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(""))))