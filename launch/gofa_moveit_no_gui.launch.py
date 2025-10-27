from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def load_file(package_name, relative_path):
    file_path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(file_path, 'r') as file:
        return file.read()

def generate_launch_description():
    # Load files
    urdf_content = load_file('gofa_moveit_config', 'urdf/gofa_with_carrier.urdf')
    srdf_content = load_file('gofa_moveit_config', 'srdf/gofa_with_carrier.srdf')
    controllers_yaml = os.path.join(
        get_package_share_directory('gofa_moveit_config'),
        'config',
        'controllers.yaml'
    )

    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
        ),

        # Move Group (planning)
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': urdf_content},
                {'robot_description_semantic': srdf_content},
                controllers_yaml
            ]
        ),
    ])

