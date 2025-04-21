import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('qube_description') # Corrected package name
    
    # Path to the URDF file
    urdf_path = os.path.join(pkg_share, 'urdf', 'qube.urdf.xacro')
    
    # Declare use_sim_time as a launch argument
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Create robot description parameter using Command substitution
    robot_description_param = {
        'robot_description': Command(['xacro ', urdf_path]),
        'use_sim_time': use_sim_time
    }
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param],
    )
    
    # Joint state publisher with GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )
    
    # RViz
    rviz_config_path = os.path.join(pkg_share, 'config', 'qube_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen',
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_arg,# Declare the use_sim_time argument
        robot_state_publisher,# Robot state publisher
        joint_state_publisher_gui, # Joint state publisher GUI
        rviz, # RViz node
    ])
