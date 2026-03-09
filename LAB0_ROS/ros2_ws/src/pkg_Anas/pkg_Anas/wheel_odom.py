import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
import tf_transformations

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        # Robot parameters
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_base', 0.30)
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Previous angles and time
        self.last_left_angle = 0.0
        self.last_right_angle = 0.0
        self.last_time = self.get_clock().now()

        # Publishers and broadcasters
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

    def joint_callback(self, msg):
        current_time = self.get_clock().now()

        # Delta angles (radians)
        d_theta_l = msg.position[0] - self.last_left_angle
        d_theta_r = msg.position[1] - self.last_right_angle

        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Distances
        ds_l = self.wheel_radius * d_theta_l
        ds_r = self.wheel_radius * d_theta_r

        # Midpoint integration
        ds = (ds_r + ds_l) / 2.0
        dpsi = (ds_r - ds_l) / self.wheel_base

        # Update pose
        mid_yaw = self.yaw + dpsi / 2.0
        self.x += ds * math.cos(mid_yaw)
        self.y += ds * math.sin(mid_yaw)
        self.yaw += dpsi

        # Velocities
        v = ds / dt
        omega = dpsi / dt

        # Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Pose
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Twist
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Update last values
        self.last_left_angle = msg.position[0]
        self.last_right_angle = msg.position[1]
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
