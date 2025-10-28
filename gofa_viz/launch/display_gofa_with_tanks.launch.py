from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # URDF file path
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'gofa_ws',
        'src',
        'ifpt_gofa',
        'urdf',
        'gofa_with_carrier.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Tank Publisher
        Node(
            package='gofa_viz',
            executable='add_tanks',
            name='add_tanks',
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                os.getenv('HOME'),
                'gofa_ws',
                'src',
                'gofa_viz',
                'config',
                'display.rviz'
            )]
        ),
    ])
