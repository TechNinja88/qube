from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.substitutions import (
    PathJoinSubstitution,
)
import xacro

def generate_launch_description():
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare("qube_driver"), "config", "joint_controllers.yaml"]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ParameterFile(initial_joint_controllers, allow_substs=True),
        ],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ]
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "velocity_controller"
        ]
    )

    nodes_to_start = [
        control_node,
        controller_spawner,
        joint_state_spawner,
    ]

    return LaunchDescription(nodes_to_start)
