import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import math

class EncoderDriver(Node):
    def __init__(self):
        super().__init__('encoder_driver')
        
        self.declare_parameter('ticks_per_rev', 2048)
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/wheel_ticks',
            self.ticks_callback,
            10)
        
        self.left_ticks = 0
        self.right_ticks = 0
        self.joint_names = ['left_wheel_joint', 'right_wheel_joint']
        
    def ticks_callback(self, msg):
        self.left_ticks = msg.data[0]
        self.right_ticks = msg.data[1]
        
        # Convert ticks to radians: θ = 2π * ticks / ticks_per_rev
        left_angle = 2 * math.pi * self.left_ticks / self.ticks_per_rev
        right_angle = 2 * math.pi * self.right_ticks / self.ticks_per_rev
        
        # Publish JointState
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = [left_angle, right_angle]
        
        self.publisher_.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    encoder_driver = EncoderDriver()
    rclpy.spin(encoder_driver)
    encoder_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
