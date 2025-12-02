import rclpy
import numpy as np
from numpy.typing import NDArray
from dataclasses import dataclass
import threading
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math

@dataclass
class VehicleData:
    # All are in meter and radian for max_steering
    length: float = 0.50
    width: float = 0.270
    radius: float = 0.0
    max_steering: float = 0.5236
    max_speed: float = 22.88
    def __post_init__(self):
        self.radius = math.sqrt((self.length/2)**2 + (self.width/2)**2) + 0.2 # safety_margin

@dataclass
class LidarData:
    # Ranges are in m, and Angles are in rad
    ranges: np.ndarray = None
    angle_min: float = 0.0
    angle_max: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.0
    range_max: float = 0.0

    def __post_init__(self):
        self.ranges = np.nan_to_num(
            self.ranges,
            nan=0.0,
            posinf=self.range_max,
            neginf=0.0
        )
    
    @classmethod
    def from_laserscan(cls, msg: LaserScan):
        """
        Transforming LaserScan from LIDAR to Python object.
        """
        return cls(
            ranges = np.array(msg.ranges),
            angle_min = msg.angle_min,
            angle_max = msg.angle_max,
            angle_increment = msg.angle_increment,
            range_min = msg.range_min,
            range_max = msg.range_max,
        )
    

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # LIDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.lidar_callback,
            10
        )
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
        # Timer for control loop
        timer_period = 1/40  # 40 Hz
        self.create_timer(timer_period, self.control_loop)

        self.lidar_data = None
        self.vehicle = VehicleData()
    
    def lidar_callback(self, msg: LaserScan) -> LidarData:
        """
        Convert LaserScan to LidarData (Python class object).
        """
        self.lidar_data = LidarData.from_laserscan(msg)

    def lidar_to_vector(self, lidar: tuple[np.float32, np.float32]) -> tuple[np.float32, np.float32]:
        """
        Retrieve vector components from given polar components.
        """
        dist, angle = lidar
        return dist * math.cos(angle), dist * math.sin(angle)

    def euclidean_dist(self, center: tuple[np.float32, np.float32], 
                       target: tuple[np.float32, np.float32]) -> np.float32:
        """
        Compute the Euclidean distance between two vectors.
        """
        x1, y1 = self.lidar_to_vector(center)
        x2, y2 = self.lidar_to_vector(target)
        return math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2)

    def put_safety_bubble(self, range_data: NDArray[np.float32], 
                          angle_data: NDArray[np.float32], rad: np.float32) -> NDArray[np.float32]:
        """
        Set all points lies in the bubble to 0.
        """
        _len = range_data.shape[0]
        # Get the closest point
        center = range_data.argmin()
        center_angle = angle_data[center]
        center_range = range_data[center]
        range_data[center] = 0.0
        # To the left
        for i in range(center-1, max(0, center-3), -1):
            target_dist, target_angle = range_data[i], angle_data[i]
            dist = self.euclidean_dist((center_range, center_angle), (target_dist, target_angle))
            if dist <= rad:
                range_data[i] = 0.0
        # To the right
        for i in range(center+1, min(_len, center+3), 1):
            target_dist, target_angle = range_data[i], angle_data[i]
            dist = self.euclidean_dist((center_range, center_angle), (target_dist, target_angle))
            if dist <= rad:
                range_data[i] = 0.0
        
        return range_data

    def filter_arc(self, range_data: NDArray[np.float32], 
                   angle_data: NDArray[np.float32], rad: np.float32) -> NDArray[np.float32]:
        """
        Filtering the ranges by zeroing out any points that are further away than a specified radius.
        """
        _len = range_data.shape[0]
        for i in range(_len):
            if range_data[i] > rad:
                range_data[i] = 0.0
        return range_data

    def find_longest_sequence(self, range_data: NDArray[np.float32]):
        """
        Find the max gap (max sequence of consecutive nonzeros of the range).
        Returns (start_gap, gap_size)
        """
        _len = range_data.shape[0]
        max_start, max_size, current_start, current_size = 0, 0, 0 ,0
        for i in range(_len):
            if range_data[i] > 2.0:
                if current_size == 0:
                    current_start = i
                current_size += 1
            else:
                if current_size > max_size:
                    max_size = current_size
                    max_start = current_start
                current_size = 0
        if current_size > max_size:
            max_size = current_size
            max_start = current_start
        
        return max_start, max_size
        
    
    def compute_direction(self):
        """
        Determine the steering and throttle commands.
        """
        if not self.lidar_data:
            return 0.0, 0.1
        lidar_data = self.lidar_data
        ranges = np.asarray(lidar_data.ranges, dtype=np.float32)
        angles = np.linspace(lidar_data.angle_min, lidar_data.angle_max, num=len(ranges))
        # Filter by bubble
        bubble_ranges = self.put_safety_bubble(ranges.copy(), angles, self.vehicle.radius)
        # Filter by radius
        filtered_ranges = self.filter_arc(bubble_ranges, angles, 10)
        gap_start, gap_size = self.find_longest_sequence(filtered_ranges)
        gap = angles[gap_start:gap_start+gap_size+1]

        steering = 0.0
        if gap.shape[0]:
            steering = np.mean(gap)

        # Go fast if straight, slow if turning
        steer_severity = abs(steering)/self.vehicle.max_steering
        steer_severity = min(1.0, steer_severity) # Extra penalty

        throttle = (0.3 - steer_severity)
        throttle = max(0.1, throttle)
        return steering, throttle
        

    def control_loop(self):
        steering_cmd = Float32()
        throttle_cmd = Float32()

        steering_cmd.data, throttle_cmd.data = self.compute_direction()

        self.steering_pub.publish(steering_cmd)
        self.throttle_pub.publish(throttle_cmd)
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()