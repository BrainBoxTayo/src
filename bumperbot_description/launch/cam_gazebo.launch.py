import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bumperbot_description = get_package_share_directory("bumperbot_description")
    
    
    use_follower_arg = DeclareLaunchArgument(
        "use_follower",
        default_value="True",
        description="Whether to use the follower node or not"
    )

    
    use_follower = LaunchConfiguration("use_follower")

    # world_name_arg = DeclareLaunchArgument(name="world_name", default_value="empty")

    camera_model_arg = DeclareLaunchArgument(name="camera_model", default_value=os.path.join(
                                        bumperbot_description, "urdf", "camera.xacro"
                                        ),
                                      description="Absolute path to camera description file"
    )

    model_path = str(Path(bumperbot_description).parent.resolve())
    model_path += pathsep + os.path.join(bumperbot_description, "models")
    gazebo_resource_path = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", model_path)

    
    camera_description = ParameterValue(Command([
            "xacro ",
            LaunchConfiguration("camera_model"),
            # " is_ignition:=",
            # is_ignition
        ]),
        value_type=str
    )

    world_path = PathJoinSubstitution([
        bumperbot_description,
        "worlds",
        PythonExpression(expression=["'", LaunchConfiguration("world_name"), "'", " + '.world'"])
    ])


    camera_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/camera",
        parameters=[{"robot_description": camera_description}]
    )


    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace="/camera",
        arguments=["-topic", "robot_description",
                   "-name", "camera1",
                   "-x", "0.0",
                   "-y", "0.0",
                   "-z", "0.0"],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace="/camera",
        arguments=[
            "/camera_north/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_south/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_east/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera_west/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
    )

    gz_ros2_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/camera_north/image_raw",
            "/camera_south/image_raw",
            "/camera_east/image_raw",
            "/camera_west/image_raw",
            ]
    )

    detector_node_surveillance_north = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ball_tracker"),
            "launch",
            "ball_tracker.launch.py"),
        launch_arguments=[
            ("detect_only", "true"),
            ("namespace", "north"),
            ("follow_only", "false"),
            ("tune_detection", "false"),
            ("use_sim_time", "true"),
            ("image_topic", "/camera_north/image_raw"),
            ("cmd_vel_topic", "null"),
            ("enable_3d_tracker", "true"),
            ("params_file", os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params_camera_north.yaml"))
        ],
        condition=IfCondition(use_follower)
    )
    detector_node_surveillance_south = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ball_tracker"),
            "launch",
            "ball_tracker.launch.py"),
        launch_arguments=[
            ("detect_only", "true"),
            ("follow_only", "false"),
            ("namespace", "south"),
            ("tune_detection", "false"),
            ("use_sim_time", "true"),
            ("image_topic", "/camera_south/image_raw"),
            ("cmd_vel_topic", "null"),
            ("enable_3d_tracker", "true"),
            ("params_file", os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params_camera_south.yaml"))
        ],
        condition=IfCondition(use_follower)
    )
    detector_node_surveillance_west = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ball_tracker"),
            "launch",
            "ball_tracker.launch.py"),
        launch_arguments=[
            ("detect_only", "true"),
            ("follow_only", "false"),
            ("namespace", "west"),
            ("tune_detection", "false"),
            ("use_sim_time", "true"),
            ("image_topic", "/camera_west/image_raw"),
            ("cmd_vel_topic", "null"),
            ("enable_3d_tracker", "true"),
            ("params_file", os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params_camera_west.yaml"))
        ],
        condition=IfCondition(use_follower)
    )
    detector_node_surveillance_east = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ball_tracker"),
            "launch",
            "ball_tracker.launch.py"),
        launch_arguments=[
            ("detect_only", "true"),
            ("follow_only", "false"),
            ("tune_detection", "false"),
            ("namespace", "east"),
            ("use_sim_time", "true"),
            ("image_topic", "/camera_north/image_raw"),
            ("cmd_vel_topic", "null"),
            ("enable_3d_tracker", "true"),
            ("params_file", os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params_camera_east.yaml"))
        ],
        
        condition=IfCondition(use_follower)
    )

    return LaunchDescription([
        use_follower_arg,
        camera_model_arg,
        gazebo_resource_path,
        camera_state_publisher_node,
        gz_spawn_entity,
        gz_ros2_bridge,
        gz_ros2_image_bridge,
        detector_node_surveillance_west,
        detector_node_surveillance_east,
        detector_node_surveillance_north,
        detector_node_surveillance_south
    ])