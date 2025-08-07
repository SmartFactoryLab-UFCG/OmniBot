from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():
    # Declare args for the launch file
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.03",
    )
    wheel_distance_arg = DeclareLaunchArgument(
        "wheel_distance",
        default_value="0.085",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_distance = LaunchConfiguration("wheel_separation")

    # Spawn the controller manager for ros2 control
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Spawn the wheel controller manager for ros2 control
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                ]
    )

    # Spawn the kinematics controller
    omni_kinematics = Node(
        package='omnibot_controllers',
        executable='omni_kinematics',
        parameters=[
            {'wheel_radius':wheel_radius,
             'wheel_distance':wheel_distance,
             'use_sim_time':use_sim_time}]
    )
  
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,
            wheel_radius_arg,
            wheel_distance_arg,
            use_sim_time_arg,
            omni_kinematics
        ]
    )