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
        timer_period = 1/40 #20 Hz
        self.create_timer(timer_period, self.control_loop)
        self.create_timer(timer_period, self.tf_callback)

        # Class Variables
        self.x, self.y, self.z = 0.0, 0.0, 0.0
        self.q = Quaternion()
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        self.paths = []
        self.lookahead = 1.0
        self.lookahead_gain = 0.5
        self.v = 0.03
        self.deltas = []
        self.avg_filter = 5.0

        # Car variable
        self.car_length = 4.5

        
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
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF not ready: {ex}", throttle_duration_sec=2.0)
        except tf2_ros.LookupException:
            self.get_logger().warn("Transform not available yet")
        except tf2_ros.ExtrapolationException:
            pass

    def path_callback(self, msg: Path):
        self.paths = []
        for pose_stamped in msg.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            self.paths.append([x,y])

    @staticmethod
    def distance(point1, point2):
        return math.sqrt((point2[1]-point1[1])**2 + (point2[0] - point1[0])**2)

    def find_target(self):
        #candidates = []
        #for px, py in self.paths:
        #    dx = px - self.x
        #    dy = py - self.y
        #
        #    # Find the rotation angle to that point
        #    x_r = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        #    y_r = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        #
        #    # Only consider points in front (x_r > 0)
        #    dist = math.hypot(dx, dy)
        #    if dist >= self.lookahead and x_r > 0:
        #        candidates.append((dist, (px, py)))

        position_array = np.array([[self.x, self.y]]*len(self.paths))
        distances_to_position = np.linalg.norm(np.abs(position_array-self.paths), axis=1)
        nearest_idx = np.argmin(distances_to_position)
        dist = 0
        i = nearest_idx
        while (dist < self.lookahead):
            i = (i+1)%len(self.paths)
            dist = math.sqrt((self.paths[nearest_idx][1]-self.paths[i][1])**2 + (self.paths[i][0]-self.paths[nearest_idx][0])**2)
        point_infront = self.paths[i]
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = point_infront[0]
        msg.point.y = point_infront[1]
        msg.point.z = 0.0
        self.debug_pub.publish(msg)
        return point_infront
        #if candidates:
        #    candidates.sort(key=lambda c: c[0])
        #    msg = PointStamped()
        #    msg.header.frame_id = "map"
        #    msg.header.stamp = self.get_clock().now().to_msg()
        #    msg.point.x = candidates[0][1][0]
        #    msg.point.y = candidates[0][1][1]
        #    msg.point.z = 0.0
        #    self.debug_pub.publish(msg)
        #    return candidates[0][1]

        #return None, None

    def compute_direction(self):
        if not self.paths:
            return 0.0, 0.05  # fallback

        # Find target point along path
        target = self.find_target()
        if target is None:
            return 0.0, self.v

        _px, _py = target
        dx = _px - self.x
        dy = _py - self.y

        # Transform target into car frame
        x_r = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        y_r = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        # Dynamic lookahead: scale with speed
        lookahead_distance = max(self.lookahead, self.v * self.lookahead_gain)
        ld = max(math.hypot(dx, dy), lookahead_distance)

        # Pure Pursuit curvature
        alpha = math.atan2(y_r, x_r)
        delta = math.atan2(2 * self.car_length * math.sin(alpha), ld)

        # Exponential smoothing for shakiness
        if not hasattr(self, 'delta_filtered'):
            self.delta_filtered = delta
        smoothing = 0.3  # adjust 0.0-1.0, lower = faster reaction
        self.delta_filtered = (1 - smoothing) * delta + smoothing * self.delta_filtered

        # Return filtered steering and velocity
        return self.delta_filtered, self.v

    
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

        
