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
    wheel1_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel1_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                ]
    )
    wheel2_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel2_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                ]
    )
    wheel3_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["wheel3_controller", 
                   "--controller-manager", 
                   "/controller_manager"
                ]
    )    
    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel1_controller_spawner,
            wheel2_controller_spawner,
            wheel3_controller_spawner
        ]
    )