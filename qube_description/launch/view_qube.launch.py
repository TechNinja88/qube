import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler

def generate_launch_description():
    # Declare the launch arguments
    declare_baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication with Qube'
    )
    
    declare_device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyACM1',
        description='Device path for Qube'
    )
    
    declare_simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Whether to run in simulation mode'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Get the launch configuration values
    baud_rate = LaunchConfiguration('baud_rate')
    device = LaunchConfiguration('device')
    simulation = LaunchConfiguration('simulation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package directories
    qube_bringup_pkg = get_package_share_directory('qube_bringup')
    
    # Path to the controlled URDF file (with ROS2 Control)
    controlled_urdf_path = os.path.join(qube_bringup_pkg, 'urdf', 'controlled_qube.urdf.xacro')
    
    # Create robot description parameter using Command substitution with parameters
    robot_description_param = {
        'robot_description': Command([
            'xacro ', controlled_urdf_path,
            ' baud_rate:=', baud_rate,
            ' device:=', device,
            ' simulation:=', simulation
        ]),
        'use_sim_time': use_sim_time
    }
    
    # Robot state publisher (processes the URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )
    
    # Controller manager for ROS2 Control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description_param,
            {'update_rate': 100}  # Hz
        ],
        output='screen',
    )
    
    # RViz for visualization
    rviz_config_path = os.path.join(qube_bringup_pkg, 'config', 'qube.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen',
    )
    
    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Velocity controller (for controlling the Qube)
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay starting controllers until after the robot_state_publisher has started
    controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_state_publisher,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    velocity_controller_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )
    
    # Return the launch description
    return LaunchDescription([
        # Arguments
        declare_baud_rate_arg,
        declare_device_arg,
        declare_simulation_arg,
        declare_use_sim_time_arg,
        
        # Nodes
        robot_state_publisher,
        controller_manager,
        rviz,
        
        # Controllers with timing dependencies
        controller_delay,
        velocity_controller_delay,
    ])
