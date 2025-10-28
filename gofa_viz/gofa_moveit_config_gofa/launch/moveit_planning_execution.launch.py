from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_file_name = 'gofa_with_carrier.urdf'
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('gofa_viz'), 'urdf', urdf_file_name]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('gofa_moveit_config_gofa'), 'config', 'moveit.rviz']
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_path],
        ),

        # Move Group
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
    ])
