from launch import LaunchDescription
<<<<<<< HEAD
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    qube_driver_dir = get_package_share_directory('qube_driver')
    qube_bringup_dir = get_package_share_directory('qube_bringup')

    # Path to xacro and launch files
    xacro_file = os.path.join(qube_bringup_dir, 'urdf', 'controlled_qube.urdf.xacro')
    qube_driver_launch = os.path.join(qube_driver_dir, 'launch', 'qube_driver.launch.py')

    return LaunchDescription([
        # Kjør qube_driver-pakken
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(qube_driver_launch)
        ),

        # Robot state publisher med URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }],
            arguments=[xacro_file]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(qube_bringup_dir, 'rviz', 'view_config.rviz')],
        )
    ])

=======
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    pkg_qube_bringup = FindPackageShare('qube_bringup')
    
    # Launch arguments
    simulation = LaunchConfiguration('simulation')
    
    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Use simulation instead of real hardware'
    )
    
    # URDF file
    urdf_file = PathJoinSubstitution([pkg_qube_bringup, 'urdf', 'controlled_qube.urdf.xacro'])
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', urdf_file,
                ' simulation:=', simulation
            ])
        }],
        output='screen',
    )
    
    # RViz
    rviz_config_file = PathJoinSubstitution([pkg_qube_bringup, 'rviz', 'qube.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
    )
    
    # Controller manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command([
                'xacro ', urdf_file,
                ' simulation:=', simulation
            ])},
            PathJoinSubstitution([pkg_qube_bringup, 'config', 'controllers.yaml'])
        ],
        output='screen',
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    # Velocity controller spawner - with dependency
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller'],
        output='screen',
    )
    
    # Make sure controllers start in sequence
    controller_sequence = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )
    
    return LaunchDescription([
        declare_simulation,
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        controller_sequence,
        rviz_node,
    ])
>>>>>>> 330d8b6d6551b8fa5b585ec0b9beba5b58b05ad5
