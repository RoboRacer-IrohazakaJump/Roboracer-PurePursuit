import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import TransformStamped, Quaternion, Twist, PointStamped
from std_msgs.msg import Float32
import tf2_ros
import tf_transformations

import csv
import math
import numpy as np

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuiter')

        # Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriber
        self.path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        # Publisher
        # Steering
        self.steering_pub = self.create_publisher(
            Float32,
            '/autodrive/roboracer_1/steering_command',
            10
        )
        # Throttle
        self.throttle_pub = self.create_publisher(
            Float32,
            '/autodrive/roboracer_1/throttle_command',
            10
        )

        # Debugging
        self.debug_pub = self.create_publisher(
            PointStamped,
            '/debug_direction',
            10
        )
        
        # Create timer 
        timer_period = 1/40 #30 Hz
        self.create_timer(timer_period, self.control_loop)
        self.create_timer(timer_period, self.tf_callback)

        # Class Variables
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.q = Quaternion()
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.paths = []
        self.lookahead = 2
        self.v = 0.05

        
    def tf_callback(self):
        try:
            # Look up SLAM-estimated transform
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.x = t.transform.translation.x
            self.y = t.transform.translation.y
            self.z = t.transform.translation.z

            self.q = t.transform.rotation
            self.roll, self.pitch, self.yaw = tf_transformations.euler_from_quaternion([
                self.q.x, self.q.y, self.q.z, self.q.w
            ])

        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not available yet")
        except tf2_ros.ExtrapolationException:
            pass

    def path_callback(self, msg: Path):
        self.paths = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.paths.append((x,y))

    def find_target(self):
        candidates = []
        for px, py in self.paths:
            dx = px - self.x
            dy = py - self.y

            # Find the rotation angle to that point
            x_r = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
            y_r = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

            # Only consider points in front (x_r > 0)
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead and x_r > 0:
                candidates.append((dist, (px, py)))

        if candidates:
            candidates.sort(key=lambda c: c[0])
            msg = PointStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.point.x = candidates[0][1][0]
            msg.point.y = candidates[0][1][1]
            msg.point.z = 0.0
            self.debug_pub.publish(msg)
            return candidates[0][1]

        return None, None

    def compute_direction(self):
        _px, _py = self.find_target()
        print("Target: ", _px, _py)
        print("Current:", self.x, self.y)
        if _px is None:
            return 0.0, self.v
        
        # Transform the target into vector
        dx = _px - self.x
        dy = _py - self.y
        x_r = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        y_r = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        # Pure Pursuit curvature
        kappa = 2 * y_r / (self.lookahead ** 2)
        omega, v = self.v * kappa * 30, self.v
        
        # Return the value
        return omega, self.v
    
    def control_loop(self):
        steering_cmd = Float32()
        throttle_cmd = Float32()

        steering_cmd.data, throttle_cmd.data = self.compute_direction()

        self.steering_pub.publish(steering_cmd)
        self.throttle_pub.publish(throttle_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

        
