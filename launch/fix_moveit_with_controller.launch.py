from launch import LaunchDescription
from launch_ros.actions import Node
import os
import yaml

def generate_launch_description():
    home = os.getenv('HOME')

    # Paths (pointing to moveit2_ws)
    urdf_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/urdf/gofa_with_carrier.urdf')
    srdf_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/gofa.srdf')
    controllers_yaml = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/controllers.yaml')
    kinematics_yaml = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/kinematics.yaml')
    planner_config_file = os.path.join(home, 'moveit2_ws/src/gofa_moveit_config/config/ompl_planning.yaml')

    # Load URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Load SRDF
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    # Load controllers.yaml
    with open(controllers_yaml, 'r') as f:
        controllers_config = yaml.safe_load(f)

    # Load kinematics.yaml
    with open(kinematics_yaml, 'r') as f:
        kinematics_config = yaml.safe_load(f)

    # Load OMPL planner config
    with open(planner_config_file, 'r') as f:
        planner_config = yaml.safe_load(f)

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),

        # Joint State Publisher (NOT GUI - for automation)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'source_list': ['/joint_trajectory_controller/joint_states']}]
        ),

        # Controller Manager (NEW - for trajectory control)
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description}, controllers_yaml],
            output='screen',
        ),

        # Joint Trajectory Controller (NEW - for automation)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller', '-c', '/controller_manager'],
            output='screen',
        ),

        # Joint State Broadcaster (NEW)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
            output='screen',
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
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                controllers_config,
                {'robot_description_kinematics': kinematics_config},
                planner_config
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'robot_description_semantic': robot_description_semantic}
            ]
        ),

    ])
