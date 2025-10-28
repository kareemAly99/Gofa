from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gofa_viz',
            executable='add_tanks',
            name='add_tanks_node',
            output='screen'
        ),
    ])
