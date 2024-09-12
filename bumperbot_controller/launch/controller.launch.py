from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    # read value of the use_python argument
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    simple_controller_py = Node(
        package="bumperbot_controller",
        executable="simple_controller.py",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheel_separation": wheel_separation}],
        condition=IfCondition(use_python)
    )
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_py
    ])
