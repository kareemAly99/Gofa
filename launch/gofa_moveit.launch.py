from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/urdf/gofa_with_carrier.urdf'
    )

    srdf_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/srdf/gofa_with_carrier.srdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[{
                'robot_description': open(urdf_file).read(),
                'robot_description_semantic': open(srdf_file).read()
            }]
        ),
    ])

