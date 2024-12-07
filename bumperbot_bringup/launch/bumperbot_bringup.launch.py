import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="True",
        description="Whether to use SLAM or not"
    )

    use_follower_arg = DeclareLaunchArgument(
        "use_follower",
        default_value="True",
        description="Whether to use the follower node or not"
    )

    use_slam = LaunchConfiguration("use_slam")
    use_follower = LaunchConfiguration("use_follower")

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_description"),
            "launch",
            "gazebo.launch.py"
        ),
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "True"
        }.items(),
    )

    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        condition=UnlessCondition(use_slam)
    )
    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    gazebo2 = TimerAction(
        period=20.0,
        actions=[
            IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory("bumperbot_description"),
                    "launch",
                    "cam_gazebo.launch.py"
                ),
            )]
    )

    local_or_slam = TimerAction(
        period=15.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_localization",
                output="screen",
                arguments=[
                    "-d", os.path.abspath("bumperbot_ws/src/bumperbot_localization/rviz/global_localization.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=UnlessCondition(use_slam)
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz_slam",
                output="screen",
                arguments=[
                    "-d", os.path.abspath("bumperbot_ws/src/bumperbot_mapping/rviz/slam.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(use_slam)),
            IncludeLaunchDescription(
                os.path.join(get_package_share_directory("nav2_bringup"),
                             "launch", "navigation_launch.py"),
                launch_arguments={
                    "use_sim_time": "True",
                }.items()
            )

        ]
    )

    detector_node = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ball_tracker"),
            "launch",
            "ball_tracker.launch.py"),
        launch_arguments=[
            ("detect_only", "false"),
            ("follow_only", "false"),
            ("tune_detection", "false"),
            ("use_sim_time", "true"),
            ("image_topic", "/bumperbot_camera/image_raw"),
            ("cmd_vel_topic", "/nav_vel_tracker"),
            ("enable_3d_tracker", "true"),
            ("params_file", os.path.join(get_package_share_directory("ball_tracker"), "config", "ball_tracker_params_bumperbot.yaml"))
        ],
        condition=IfCondition(use_follower)
    )
 

    return LaunchDescription([
        use_slam_arg,
        use_follower_arg,
        gazebo,
        gazebo2,
        controller,
        joystick,
        localization,
        slam,
        local_or_slam,
        detector_node,
        
    ])
