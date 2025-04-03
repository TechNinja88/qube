from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    target_position = LaunchConfiguration('target_position')
    kp = LaunchConfiguration('kp')
    ki = LaunchConfiguration('ki')
    kd = LaunchConfiguration('kd')
    max_velocity = LaunchConfiguration('max_velocity')
    joint_name = LaunchConfiguration('joint_name')
    
    declare_target_position = DeclareLaunchArgument(
        'target_position',
        default_value='1.57',  # ~90 deg
        description='Target position for the motor joint in radians'
    )
    
    declare_kp = DeclareLaunchArgument(
        'kp',
        default_value='2.0',
        description='Proportional gain'
    )
    
    declare_ki = DeclareLaunchArgument(
        'ki',
        default_value='0.2',
        description='Integral gain'
    )
    
    declare_kd = DeclareLaunchArgument(
        'kd',
        default_value='0.1',
        description='Derivative gain'
    )
    
    declare_max_velocity = DeclareLaunchArgument(
        'max_velocity',
        default_value='6.0',
        description='Maximum velocity command'
    )
    
    declare_joint_name = DeclareLaunchArgument(
        'joint_name',
        default_value='motor_joint',  
        description='Name of the joint to control'
    )
    
    # PID Controller Node
    pid_controller_node = Node(
        package='qube_controller',
        executable='pid_controller',
        name='pid_controller',
        parameters=[{
            'target_position': target_position,
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'max_velocity': max_velocity,
            'joint_name': joint_name
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_target_position,
        declare_kp,
        declare_ki,
        declare_kd,
        declare_max_velocity,
        declare_joint_name,
        pid_controller_node
    ])
