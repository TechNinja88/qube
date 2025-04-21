from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # Get directories
    qube_bringup_pkg_share = get_package_share_directory('qube_bringup')# Path to the bringup package
    qube_driver_pkg_share = get_package_share_directory('qube_driver')# Path to the driver package
    # Launch configuration variables
    simulation = LaunchConfiguration('simulation')# Simulation mode
    use_sim_time = LaunchConfiguration('use_sim_time')# Use simulation time
    # Launch arguments
    declare_simulation_cmd = DeclareLaunchArgument(
        name='simulation',
        default_value='false',
        description='Use simulation if true'
    )
    # Launch arguments for simulation time
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    # Build the URDF model
    urdf_model_path = os.path.join(qube_bringup_pkg_share, 'urdf', 'controlled_qube.urdf.xacro') # Path to the URDF model
    robot_description_content = Command([
        'xacro ', urdf_model_path,
        ' simulation:=', simulation
    ])
    # robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    # Launch qube_driver
    qube_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(qube_driver_pkg_share, 'launch', 'qube_driver.launch.py') # Path to the driver launch file
        ),
        launch_arguments={
            'simulation': simulation    # Simulation mode
        }.items()
    )
    # RViz configuration
    rviz_config_file = os.path.join(qube_bringup_pkg_share, 'rviz', 'qube.rviz') # Path to the RViz configuration file
    # Create RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [] # Check if the RViz config file exists
    )
    # Launch description
    return LaunchDescription([
        declare_simulation_cmd,
        declare_use_sim_time_cmd,
        robot_state_publisher_node,
        qube_driver_launch,
        rviz_node
    ])
