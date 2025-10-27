from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Paths
    urdf_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/urdf/gofa_with_carrier.urdf'
    )
    srdf_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/srdf/gofa_with_carrier.srdf'
    )
    controllers_yaml = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/config/controllers.yaml'
    )
    rviz_config_file = os.path.join(
        os.getenv('HOME'),
        'moveit2_ws/src/gofa_moveit_config/config/config.rviz'
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # Joint state publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Move Group (planning)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': open(urdf_file).read()},
                {'robot_description_semantic': open(srdf_file).read()},
                controllers_yaml
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])

