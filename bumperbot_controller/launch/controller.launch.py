from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def noisy_controller(context, *args, **kwargs):
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    wheel_separation = float(LaunchConfiguration(
        "wheel_separation").perform(context))
    wheel_radius_error = float(LaunchConfiguration(
        "wheel_radius_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration(
        "wheel_separation_error").perform(context))

    noisy_controller_py = Node(
        package="bumperbot_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error}
        ]
    )
    return [noisy_controller_py]


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

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="False"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    # read value of the use_python argument
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    

    wheel_controller_spawner = TimerAction(
        period=10.0,  # delay by 30 seconds
        actions=[
        Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "bumperbot_controller",
                "--controller-manager",
                "/controller_manager",
            ],
            condition=UnlessCondition(use_simple_controller)
        ),
        
         ]

    )

    simple_controller = GroupAction(

        actions=[
            Node(
                package="bumperbot_controller",
                executable="simple_controller.py",
                parameters=[{"wheel_radius": wheel_radius,
                             "wheel_separation": wheel_separation}],
                condition=IfCondition(use_python)
            ),

            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ]
            )
        ],
        condition=IfCondition(use_simple_controller)
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription([
        use_python_arg,
        use_simple_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        simple_controller,
        wheel_controller_spawner,
        noisy_controller_launch
    ])
