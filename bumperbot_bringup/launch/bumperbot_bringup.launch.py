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

    use_slam = LaunchConfiguration("use_slam")

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

    # safety_stop = Node(
    #     package="bumperbot_utils",
    #     executable="safety_stop",
    #     output="screen"
    # )

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
                launch_arguments={"use_sim_time": "True"}.items()
            )

        ]
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        joystick,
        # safety_stop,
        localization,

        slam,
        local_or_slam,
    ])
