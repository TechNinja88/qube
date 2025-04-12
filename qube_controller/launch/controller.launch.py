from launch import LaunchDescription # Import LaunchDescription
from launch_ros.actions import Node # Import Node
from launch.substitutions import LaunchConfiguration # Import LaunchConfiguration
from launch.actions import DeclareLaunchArgument # Import DeclareLaunchArgument

def generate_launch_description():# Function to generate launch description
    # Launch arguments
    target_position = LaunchConfiguration('target_position') # Target position for the motor joint
    kp = LaunchConfiguration('kp') # Proportional gain
    ki = LaunchConfiguration('ki') # Integral gain
    kd = LaunchConfiguration('kd') # Derivative gain
    max_velocity = LaunchConfiguration('max_velocity')
    joint_name = LaunchConfiguration('joint_name') # Name of the joint to control
    deadband = LaunchConfiguration('deadband')   # Deadband compensation for motor
    
    declare_target_position = DeclareLaunchArgument( # Added target position declaration
        'target_position',
        default_value='1.57',  # ~90 deg 3.16 (180 deg) 3pi/2 (270 deg) 2pi(360 deg)
        description='Target position for the motor joint in radians' # Description of the target position
    )
    
    declare_kp = DeclareLaunchArgument( # Added kp declaration
        'kp',
        default_value='2.0',
        description='Proportional gain'
    )
    
    declare_ki = DeclareLaunchArgument( # Added ki declaration
        'ki',
        default_value='0.2',
        description='Integral gain'
    )
    
    declare_kd = DeclareLaunchArgument( # Added kd declaration
        'kd',
        default_value='0.1',
        description='Derivative gain'
    )
    
    declare_max_velocity = DeclareLaunchArgument( # Added max velocity declaration
        'max_velocity',
        default_value='6.0',
        description='Maximum velocity command'
    )
    
    declare_joint_name = DeclareLaunchArgument( # Added joint name declaration
        'joint_name',
        default_value='motor_joint',   # Default joint name
        description='Name of the joint to control'
    )
    
    declare_deadband = DeclareLaunchArgument(  # Added deadband declaration
        'deadband',
        default_value='0.5',
        description='Deadband compensation for motor'
    )
    
    # PID Controller Node
    pid_controller_node = Node(
        package='qube_controller', # Package name
        executable='pid_controller', # Executable name
        name='pid_controller', # Node name
        parameters=[{   # Added parameters for the node
            'target_position': target_position,
            'kp': kp,
            'ki': ki,
            'kd': kd,
            'max_velocity': max_velocity,
            'joint_name': joint_name,
            'deadband': deadband  # Added deadband parameter
        }],
        output='screen'
    )
    
    return LaunchDescription([ # Return the launch description
        declare_target_position,
        declare_kp,
        declare_ki,
        declare_kd,
        declare_max_velocity,
        declare_joint_name,
        declare_deadband,  # Added deadband to the launch description
        pid_controller_node
    ])
