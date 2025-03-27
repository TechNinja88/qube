import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('qube_description')
    
    # Path to the URDF file
    urdf_path = os.path.join(pkg_share, 'urdf', 'qube.urdf.xacro')
    
    # Create robot description parameter using Command substitution
    robot_description = {'robot_description': Command(['xacro ', urdf_path])}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'qube_view.rviz')] 
                  if os.path.exists(os.path.join(pkg_share, 'config', 'qube_view.rviz')) 
                  else [],
        output='screen',
    )
    
    # Return the launch description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz,
    ])
