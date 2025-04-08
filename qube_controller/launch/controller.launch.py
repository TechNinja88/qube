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
    deadband = LaunchConfiguration('deadband')
    
    # Declare launch arguments with default values
    declare_target_position = DeclareLaunchArgument(
        'target_position',
        default_value='1.57',
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
    
    declare_deadband = DeclareLaunchArgument(
        'deadband',
        default_value='0.8',
        description='Motor deadband compensation'
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
            'deadband': deadband
        }],
        output='screen'
    )
    
    return LaunchDescription([
        declare_target_position,
        declare_kp,
        declare_ki,
        declare_kd,
        declare_max_velocity,
        declare_deadband,
        pid_controller_node
    ])
