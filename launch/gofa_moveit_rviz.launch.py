from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    home = os.getenv('HOME')

    # Paths
    urdf_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/urdf/gofa_with_carrier.urdf')
    srdf_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/gofa.srdf')
    controllers_yaml = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/controllers.yaml')
    rviz_config_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/config.rviz')

    # Load controllers.yaml
    with open(controllers_yaml, 'r') as f:
        controllers_config = yaml.safe_load(f)

    # Read URDF and SRDF contents
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic},
                controllers_config,
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'}
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

    ])
