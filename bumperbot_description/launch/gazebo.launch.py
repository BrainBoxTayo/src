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

    # this function is used to generate the launch description
    # basically everything we want to launch, all nodes, applications, processes etc..
    # it is statutory for a ros2 launch file

    bumperbot_description = get_package_share_directory(
        "bumperbot_description")
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")

    model_path = os.path.join(bumperbot_description, "models") #/home/$USER/bumperbot_ws/src/bumperbot_description/models
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")
    """
    GAZEBO_MODEL_PATH: colon-separated set of directories where Gazebo will search for models

    GAZEBO_RESOURCE_PATH: colon-separated set of directories where Gazebo will search for other resources such as world and media files.

    GAZEBO_MASTER_URI: URI of the Gazebo master. This specifies the IP and port where the server will be started and tells the clients where to connect to.

    GAZEBO_PLUGIN_PATH: colon-separated set of directories where Gazebo will search for the plugin shared libraries at runtime.

    GAZEBO_MODEL_DATABASE_URI: URI of the online model database where Gazebo will download models from.
    """
    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path) # used to set the environment variable for gazebo to find the models

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

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")))
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "bumperbot", "-topic", "robot_description"],
        output="screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])
