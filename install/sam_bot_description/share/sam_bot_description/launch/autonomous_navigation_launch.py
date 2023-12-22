# autonomous_navigation_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_robot_navigation_package',
            executable='your_amcl_node_executable',
            output='screen',
            parameters=[{'use_sim_time': True}]  # Add other parameters as needed
        ),
        Node(
            package='your_object_detection_package',
            executable='your_object_detection_node_executable',
            output='screen',
            parameters=[{'use_sim_time': True}]  # Add other parameters as needed
        ),
        Node(
            package='your_custom_navigation_node_package',
            executable='your_custom_navigation_node_executable',
            output='screen',
            parameters=[{'use_sim_time': True}]  # Add other parameters as needed
        ),
    ])
