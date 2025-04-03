from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    pkg_qube_bringup = FindPackageShare('qube_bringup')
    
    simulation = LaunchConfiguration('simulation')
    
    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='true',
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
                ' baud_rate:=115200',
                ' device:=/dev/ttyACM1',
                ' simulation:=', simulation
            ])
        }],
        output='screen',
    )
    
    # Controller manager (simplest possible configuration)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': Command([
                'xacro ', urdf_file,
                ' baud_rate:=115200',
                ' device:=/dev/ttyACM1',
                ' simulation:=', simulation
            ])},
            PathJoinSubstitution([pkg_qube_bringup, 'config', 'controllers.yaml'])
        ],
        output='screen',
    )
    
    return LaunchDescription([
        declare_simulation,
        robot_state_publisher_node,
        controller_manager_node,
    ])
