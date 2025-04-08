import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Declare parameters without validation constraints
        params = [
            ('kp', 2.0),          # Proportional gain
            ('ki', 0.2),          # Integral gain
            ('kd', 0.1),          # Derivative gain
            ('target_position', 1.57),  # Default target ~90 degrees
            ('max_velocity', 6.0),      # Maximum velocity command
            ('deadband', 0.8),          # Motor deadband compensation
            ('joint_name', 'motor_joint')  # Joint to control
        ]
        
        for name, default in params:
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)
        
        # Initialize controller state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.stable_count = 0
        
        # Create subscription and publisher
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        
        # Set parameter callback    
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info('PID controller initialized')
        self.get_logger().info(f'Target position: {self.target_position:.2f} rad ({math.degrees(self.target_position):.1f}°)')
        self.get_logger().info(f'PID gains: kp={self.kp}, ki={self.ki}, kd={self.kd}')
        self.get_logger().info(f'Max velocity: {self.max_velocity}, Deadband: {self.deadband}')
        self.get_logger().info(f'Controlling joint: {self.joint_name}')
    
    def parameters_callback(self, params):
        for param in params:
            if hasattr(self, param.name):
                old_value = getattr(self, param.name)
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Updated {param.name}: {old_value} → {param.value}")
                
        # Reset integral term when parameters change to prevent unexpected behavior
        self.integral = 0.0
        
        result = SetParametersResult()
        result.successful = True
        return result
    
    def joint_state_callback(self, msg):
        # Get joint position and velocity
        if self.joint_name not in msg.name:
            self.get_logger().warn(f'{self.joint_name} not found in joint states')
            return
            
        idx = msg.name.index(self.joint_name)
        position = msg.position[idx]
        velocity = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
        
        # Time delta calculation
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001:  # Avoid extremely small time steps
            return
            
        self.last_time = current_time
        
        # Calculate error and normalize to [-pi, pi]
        error = self.normalize_angle(self.target_position - position)
        
        # Calculate PID terms
        p_term = self.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = max(min(self.integral, 2.0), -2.0)  # Limit integral term
        i_term = self.ki * self.integral
        
        # Derivative term (using both error derivative and measured velocity)
        error_derivative = (error - self.prev_error) / dt
        d_term = self.kd * (error_derivative - velocity)
        self.prev_error = error
        
        # Calculate control signal
        control_signal = p_term + i_term + d_term
        
        # Apply deadband compensation
        if 0 < abs(control_signal) < self.deadband:
            control_signal = self.deadband * (1 if control_signal > 0 else -1)
            
        # Apply boost for significant errors when not moving
        if abs(error) > 0.2 and abs(velocity) < 0.05:
            control_signal *= 1.5
            
        # Limit to maximum velocity
        control_signal = max(min(control_signal, self.max_velocity), -self.max_velocity)
        
        # Stability detection - stop the motor when position is stable
        if abs(error) < 0.05 and abs(velocity) < 0.1:
            self.stable_count = min(self.stable_count + 1, 20)
            if self.stable_count > 10:
                control_signal = 0.0
        else:
            self.stable_count = 0
        
        # Publish command
        self.publish_command(control_signal)
        
        # Log occasionally (every ~1 second)
        if (current_time.nanoseconds // 500000000) % 2 == 0:
            self.get_logger().info(
                f'Pos: {position:.2f}, Target: {self.target_position:.2f}, ' + 
                f'Cmd: {control_signal:.2f}, P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f}'
            )
    
    def publish_command(self, value):
        msg = Float64MultiArray()
        
        # Create and configure layout
        msg.layout = MultiArrayLayout()
        
        dim = MultiArrayDimension()
        dim.label = "velocity"
        dim.size = 1
        dim.stride = 1
        
        msg.layout.dim.append(dim)
        msg.layout.data_offset = 0
        msg.data = [float(value)]
        
        self.velocity_pub.publish(msg)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    controller = PIDController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Send stop command before shutdown for safety
        controller.publish_command(0.0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
