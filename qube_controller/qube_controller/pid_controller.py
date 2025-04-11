#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import math

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        
        # Declare parameters
        self.declare_parameter('kp', 0.4)
        self.declare_parameter('ki', 0.01)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('target_position', 1.57)  # Match launch file default
        self.declare_parameter('max_velocity', 4.0)
        self.declare_parameter('deadband', 0.8)
        self.declare_parameter('joint_name', 'motor_joint')  # Add joint_name parameter
        
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
        
        # Create subscription and publisher
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.velocity_pub = self.create_publisher(
            Float64MultiArray, '/velocity_controller/commands', 10)
        
        # Set parameter callback    
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.get_logger().info(f'PID Controller initialized with:')
        self.get_logger().info(f'  kp={self.kp}, ki={self.ki}, kd={self.kd}')
        self.get_logger().info(f'  target_position={self.target_position}')
        self.get_logger().info(f'  joint_name={self.joint_name}')
    
    def parameters_callback(self, params):
        result = SetParametersResult()
        result.successful = True
        for param in params:
            old_value = getattr(self, param.name, None)
            if param.name in ['kp', 'ki', 'kd', 'target_position', 'max_velocity', 'deadband', 'joint_name']:
                setattr(self, param.name, param.value)
                self.get_logger().info(f"Parameter {param.name} changed from {old_value} to {param.value}")
                
                # Reset integral when control parameters change
                if param.name in ['kp', 'ki', 'kd']:
                    self.integral = 0.0
                    
        return result
    
    def joint_state_callback(self, msg):
        # Get joint position and velocity
        if self.joint_name not in msg.name:
            self.get_logger().warn(f'{self.joint_name} not found in {msg.name}')
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
        if dt < 0.001: return
        self.last_time = current_time
        
        # Calculate error and PID terms
        error = self.normalize_angle(self.target_position - position)
        p_term = self.kp * error
        
        self.integral += error * dt
        self.integral = max(min(self.integral, 2.0), -2.0)  # Anti-windup
        i_term = self.ki * self.integral
        
        error_derivative = (error - self.prev_error) / dt
        d_term = self.kd * (error_derivative - velocity)
        self.prev_error = error
        
        # Calculate control signal with deadband compensation
        control_signal = p_term + i_term + d_term
        
        # Apply deadband compensation
        if 0 < abs(control_signal) < self.deadband:
            control_signal = self.deadband * (1 if control_signal > 0 else -1)
            
        # Apply boost for significant errors when not moving
        if abs(error) > 0.2 and abs(velocity) < 0.05:
            control_signal *= 1.5
            
        # Limit to maximum velocity
        control_signal = max(min(control_signal, self.max_velocity), -self.max_velocity)
        
        # Stability detection
        if abs(error) < 0.05 and abs(velocity) < 0.1:
            self.stable_count = min(self.stable_count + 1, 20)
            if self.stable_count > 10:
                control_signal = 0.0
        else:
            self.stable_count = 0
        
        # Publish command
        self.publish_command(control_signal)
        
        # Log occasionally
        if (current_time.nanoseconds // 500000000) % 2 == 0:
            self.get_logger().info(f'Pos: {position:.2f}, Target: {self.target_position:.2f}, ' + 
                                  f'Cmd: {control_signal:.2f}, P={p_term:.2f}, I={i_term:.2f}, D={d_term:.2f}')
    
    def publish_command(self, value):
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
