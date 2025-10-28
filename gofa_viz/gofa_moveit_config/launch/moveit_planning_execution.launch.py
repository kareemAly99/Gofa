from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    urdf_file = LaunchConfiguration('robot_description', default='/home/kareem/gofa_ws/src/gofa_viz/urdf/gofa_with_carrier.urdf')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            default_value=urdf_file,
            description='Full path to robot URDF'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_file}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/kareem/gofa_ws/src/gofa_viz/config/gofa.rviz']
        ),
    ])
