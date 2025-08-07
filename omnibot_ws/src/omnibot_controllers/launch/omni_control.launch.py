from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():

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

    # Spawn the wheel controller for ros2 control
    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                ]
    )
  
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_controller_spawner,

        ]
    )