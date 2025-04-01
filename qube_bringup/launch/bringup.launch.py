from launch import LaunchDescription
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

