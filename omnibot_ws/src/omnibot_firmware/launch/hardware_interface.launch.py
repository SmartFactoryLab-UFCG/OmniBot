import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

ARGUMENTS = [
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
]

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create a robot_state_publisher node
    pkg_path = get_package_share_directory('omnibot_description')
    xacro_file = os.path.join(pkg_path,'urdf', 'main.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file,' is_sim:=False'])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            params,
            os.path.join(
                get_package_share_directory("omnibot_controllers"),
                "config",
                "controller_config_3w.yaml",
            ),
        ],
    )
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(controller_manager)
    return ld