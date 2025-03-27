import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'qube_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Create robot description parameter
    urdf_file = os.path.join(pkg_share, 'urdf', 'qube.urdf.xacro')
    
    # Print debug info to console
    print(f"URDF file path: {urdf_file}")
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': {'robot_description': 
                "xacro " + urdf_file},
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'source_list': []}]
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'qube_view.rviz')] if os.path.exists(os.path.join(pkg_share, 'config', 'qube_view.rviz')) else []
    )
    
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])
