#the controller works properly in simulation but not with quanser qube
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Declare parameters with better defaults based on hardware specs
        self.declare_parameter('kp', 1.2)          # Increased based on motor capability
        self.declare_parameter('ki', 0.05)         # Small but effective for steady-state error
        self.declare_parameter('kd', 0.08)         # Added to dampen oscillations
        self.declare_parameter('target_position', 1.57)  # Match launch file default
        self.declare_parameter('max_velocity', 6.0)      # max motor speed for safety
        self.declare_parameter('deadband', 0.4)          # Lower deadband for precision
        self.declare_parameter('joint_name', 'motor_joint')
    
        # System constants based on Quanser specs
        self.COUNTS_PER_REV = 2048
        self.RAD_PER_COUNT = 2 * math.pi / self.COUNTS_PER_REV
        self.MAX_MOTOR_RPM = 5400
        self.PENDULUM_LENGTH = 0.095 
        
        # Get parameters from parameter server
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.target_position = self.get_parameter('target_position').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.deadband = self.get_parameter('deadband').value
        self.joint_name = self.get_parameter('joint_name').value
        
        # Initialize controller state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None
        self.stable_count = 0
        self.last_position = None
        self.velocity_filter = [0.0] * 5  # Simple moving average filter
        
        # Create subscription and publisher
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        
        # Set parameter callback    
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(f'PID Controller optimized for Quanser Qube:')
        self.get_logger().info(f'  kp={self.kp}, ki={self.ki}, kd={self.kd}')
        self.get_logger().info(f'  target_position={self.target_position}')
    
    def parameters_callback(self, params):
        result = SetParametersResult()
        result.successful = True
        for param in params:
            old_value = getattr(self, param.name, None)
            if param.name in ['kp', 'ki', 'kd', 'target_position', 'max_velocity', 'deadband', 'joint_name']:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Parameter {param.name} changed: {old_value} → {param.value}")
                
                # Reset integral when control parameters change
                if param.name in ['kp', 'ki', 'kd', 'target_position']:
                    self.integral = 0.0
                    
        return result
    
    def joint_state_callback(self, msg):
        # Get joint position and velocity
        if self.joint_name not in msg.name:
            self.get_logger().warn(f'{self.joint_name} not found in message')
            return
            
        idx = msg.name.index(self.joint_name)
        position = msg.position[idx]
        
        # Time delta calculation
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            self.last_position = position
            return
            
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt < 0.001:  # Skip if time delta too small
            return
            
        # Calculate velocity with filtering if not provided
        if len(msg.velocity) > idx:
            velocity = msg.velocity[idx]
        else:
            # Calculate and filter velocity 
            raw_velocity = (position - self.last_position) / dt
            self.velocity_filter.pop(0)
            self.velocity_filter.append(raw_velocity)
            velocity = sum(self.velocity_filter) / len(self.velocity_filter)
            
        self.last_position = position
        self.last_time = current_time
        
        # Calculate normalized error (shortest path considering rotation)
        error = self.normalize_angle(self.target_position - position)
        
        # PID control terms
        p_term = self.kp * error
        
        # Integral with anti-windup
        self.integral += error * dt
        # Scale anti-windup based on motor capabilities
        windup_limit = 2.0 / self.kp  # Adjust based on kp
        self.integral = max(min(self.integral, windup_limit), -windup_limit)
        i_term = self.ki * self.integral
        
        # Derivative term - use measured velocity for better noise handling
        d_term = self.kd * -velocity  # Negative since velocity already indicates direction
        
        # Calculate control signal
        control_signal = p_term + i_term + d_term
        
        # Adaptive deadband compensation
        if 0 < abs(control_signal) < self.deadband:
            # Scale by position error to make deadband more aggressive near target
            deadband_scale = min(1.0, abs(error) * 5)  # Scale down when close to target
            control_signal = self.deadband * deadband_scale * (1 if control_signal > 0 else -1)
        
        # Starting torque boost for overcoming static friction
        if abs(error) > 0.2 and abs(velocity) < 0.1:
            control_signal *= 1.5
        
        # Velocity limiting - gentler near the target
        position_factor = min(1.0, abs(error) * 3.0)
        effective_max = self.max_velocity * position_factor
        control_signal = max(min(control_signal, effective_max), -effective_max)
        
        # Stability detection with hysteresis 
        if abs(error) < 0.03 and abs(velocity) < 0.05:  # Tighter thresholds based on encoder resolution
            self.stable_count = min(self.stable_count + 1, 20)
            if self.stable_count > 10:
                control_signal = 0.0  # Complete stop when stable
        else:
            self.stable_count = max(0, self.stable_count - 2)  # Faster exit from stability
        
        # Publish command
        self.publish_command(control_signal)
        
        # Log occasionally
        if (current_time.nanoseconds // 500000000) % 2 == 0:
            self.get_logger().info(
                f'Pos: {position:.3f}, Target: {self.target_position:.3f}, ' + 
                f'Error: {error:.3f}, Vel: {velocity:.3f}, ' +
                f'Cmd: {control_signal:.3f} (P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f})'
            )
    
    def publish_command(self, value):#got help from teacher based on info from this website(https://docs.ros.org/en/melodic/api/std_msgs/html/msg/MultiArrayLayout.html)
        msg = Float64MultiArray()
        msg.layout = MultiArrayLayout()
        
        dim = MultiArrayDimension()
        dim.label, dim.size, dim.stride = "velocity", 1, 1
        
        msg.layout.dim.append(dim)
        msg.layout.data_offset = 0
        msg.data = [float(value)]
        
        self.velocity_pub.publish(msg)
    
    def normalize_angle(self, angle):
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
        # Send stop command before shutdown
        controller.publish_command(0.0)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()