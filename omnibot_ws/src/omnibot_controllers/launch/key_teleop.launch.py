import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    controller_pkg = get_package_share_directory('omnibot_controllers')

    key_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        # O prefixo 'xterm -e' é um truque para abrir o nó numa nova janela de terminal.
        # Isto é altamente recomendado para o teleop_twist_keyboard para garantir
        # que ele receba os inputs do teclado corretamente.
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/key_vel')]
    )    
    
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "cmd_vel",
            "config_locks": os.path.join(controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            key_teleop,
            twist_mux_launch
        ]
    )