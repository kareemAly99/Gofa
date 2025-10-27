from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to the URDF inside your package
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/urdf/gofa_with_carrier.urdf'
    )

    # Path to RViz config (can create a new one or reuse previous)
    rviz_config_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/config/gofa.rviz'
    )

    return LaunchDescription([
        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Launch RViz2 with the config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    ])

