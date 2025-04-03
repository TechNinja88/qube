import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Declare parameters
        self.declare_parameter('kp', 2.0)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('target_position', 0.0)
        self.declare_parameter('max_velocity', 6.0)  # maximum velocity command (rad/s)
        
        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_position = self.get_parameter('target_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Initialize PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
        
        # Create subscription to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Create publisher for velocity commands
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
            
        # Create parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Log initialization success
        self.get_logger().info('PID Controller initialized with kp={}, ki={}, kd={}'.format(
            self.kp, self.ki, self.kd))
    
    def parameters_callback(self, params):
        # Update parameters when they change
        for param in params:
            if param.name == 'kp':
                self.kp = param.value
                self.get_logger().info(f"Updated kp to {self.kp}")
            elif param.name == 'ki':
                self.ki = param.value
                self.get_logger().info(f"Updated ki to {self.ki}")
            elif param.name == 'kd':
                self.kd = param.value
                self.get_logger().info(f"Updated kd to {self.kd}")
            elif param.name == 'target_position':
                self.target_position = param.value
                self.get_logger().info(f"Updated target position to {self.target_position}")
            elif param.name == 'max_velocity':
                self.max_velocity = param.value
                self.get_logger().info(f"Updated max velocity to {self.max_velocity}")
                
        # Reset integral for anti-windup when parameters change
        self.integral = 0.0
        
        # Must return a SetParametersResult, not a boolean
        result = SetParametersResult()
        result.successful = True
        result.reason = 'Parameters updated successfully'
        return result
    
    def joint_state_callback(self, msg):
        """Process joint states and compute control signal"""
        # Extract rotary joint position and velocity
        if 'motor_joint' in msg.name:  # Changed from 'rotary_joint'
            joint_idx = msg.name.index('motor_joint')  # Changed from 'rotary_joint'
            position = msg.position[joint_idx]
            velocity = msg.velocity[joint_idx] if len(msg.velocity) > joint_idx else 0.0
        else:
            self.get_logger().warn('motor_joint not found in joint states')
            return
        
        # Compute time delta
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9  # convert to seconds
        self.last_time = current_time
        
        # Compute error: normalize angle to [-pi, pi]
        error = self.normalize_angle(self.target_position - position)
        
        # PID control
        p_term = self.kp * error
        
        # Accumulate integral with anti-windup
        self.integral += error * dt
        if abs(self.integral) > 5.0:  # Anti-windup measure
            self.integral = 5.0 if self.integral > 0 else -5.0
        i_term = self.ki * self.integral
        
        # Compute derivative term (using actual velocity)
        d_term = self.kd * (-velocity)  # Negative because velocity is in the direction opposite to error reduction
        
        # Compute total control signal
        control_signal = p_term + i_term + d_term
        
        # Limit control signal to max_velocity
        if abs(control_signal) > self.max_velocity:
            control_signal = self.max_velocity if control_signal > 0 else -self.max_velocity
        
        # Publish velocity command
        velocity_cmd = Float64MultiArray()
        velocity_cmd.data = [float(control_signal)]
        self.velocity_pub.publish(velocity_cmd)
        
        # Log
        self.get_logger().info(f'Target: {self.target_position:.2f}, Position: {position:.2f}, Error: {error:.2f}, Command: {control_signal:.2f}')
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    pid_controller = PIDController()
    
    try:
        rclpy.spin(pid_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown
        pid_controller.get_logger().info('Shutting down PID Controller')
        pid_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
