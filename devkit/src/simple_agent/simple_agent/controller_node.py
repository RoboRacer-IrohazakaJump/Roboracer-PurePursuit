import rclpy
import numpy as np
from numpy.typing import NDArray
import threading
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import math

class LidarData:
    def __init__(self, msg: LaserScan):
        self.ranges = np.array(msg.ranges)
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max
    

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Subscriber
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.lidar_callback,
            10
        )

        # Publisher
        self.steering_pub = self.create_publisher(
            Float32,
            '/autodrive/roboracer_1/steering_command',
            10
        )

        self.throttle_pub = self.create_publisher(
            Float32,
            '/autodrive/roboracer_1/throttle_command',
            10
        )

        # Timer for control loop
        timer_period = 0.066  # 30 Hz
        self.create_timer(timer_period, self.control_loop)

        self.lidar_data = None
        self.top_n=90
    
    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = LidarData(msg)

    def lidar_to_vector(self, lidar):
        dist, angle = lidar
        return dist * math.cos(angle), dist * math.sin(angle)

    def eucledian_dist(self, center, target):
        x1, y1 = self.lidar_to_vector(center)
        x2, y2 = self.lidar_to_vector(target)
        return math.sqrt((x1-x2) ** 2 + (y1-y2) ** 2)

    def put_safety_bubble(self, range_data: NDArray[np.float32], angle_data: NDArray[np.float32], rad):
        _len = range_data.shape[0]
        _center = range_data.argmin()
        _center_angle = angle_data[_center]
        _center_range = range_data[_center]
        range_data[_center] = 0.0
        for i in range(_center-1, max(0, _center-3), -1):
            _target_dist, _target_angle = range_data[i], angle_data[i]
            _dist = self.eucledian_dist((_center_range, _center_angle), (_target_dist, _target_angle))
            if _dist < rad:
                range_data[i] = 0.0

        for i in range(_center+1, min(_len, _center+3), 1):
            _target_dist, _target_angle = range_data[i], angle_data[i]
            _dist = self.eucledian_dist((_center_range, _center_angle), (_target_dist, _target_angle))
            if _dist < rad:
                range_data[i] = 0.0
        
        return range_data

    def filter_arc(self, range_data: NDArray[np.float32], angle_data: NDArray[np.float32], rad):
        _len = range_data.shape[0]
        for i in range(_len):
            _dist = self.eucledian_dist((0, 0), (range_data[i], angle_data[i]))
            if _dist > rad:
                range_data[i] = rad
        return range_data

    def find_longest_sequence(self, range_data: NDArray[np.float32]):
        _len = range_data.shape[0]
        max_seq, max_i, max_temp = 0, 0, 0 
        for i in range(_len):
            if range_data[i] > 1.0:
                max_temp += 1
            else:
                max_temp = 0
            if max_temp > max_seq:
                max_seq, max_i = max_temp, i
        return max_seq, max_i
        
    
    def compute_direction(self):
        if not self.lidar_data:
            return 0.0, 0.1
        _lidar_data = self.lidar_data
        _ranges = np.asarray(_lidar_data.ranges, dtype=np.float32)
        _angle = np.arange(_lidar_data.angle_min, _lidar_data.angle_max, _lidar_data.angle_increment)
        _bubble_ranges = self.put_safety_bubble(_ranges.copy(), _angle, 5)
        _filtered_ranges = self.filter_arc(_bubble_ranges, _angle, 10)
        _gap_seq, _gap_idx = self.find_longest_sequence(_filtered_ranges)
        _gap = _angle[_gap_idx-_gap_seq+1:_gap_idx+1]
        #print("Filtered ranges")
        #print(_filtered_ranges, _filtered_ranges.shape[0])
        #print("Gap sequence and index")
        #print(_gap_seq, _gap_idx)
        #print("Gap")
        #print(_ranges[_gap_idx-_gap_seq:_gap_idx])
        #print("Range min")
        #print(_ranges.min())
        _steering = 0.0
        if _gap.shape[0]:
            _steering = sum(_gap) / _gap.shape[0]
        _throttle = 0.05 if _ranges.min() < 0.6 else 0.15
        return _steering, _throttle
        

    def control_loop(self):
        steering_cmd = Float32()
        throttle_cmd = Float32()

        steering_cmd.data, throttle_cmd.data = self.compute_direction()

        self.steering_pub.publish(steering_cmd)
        self.throttle_pub.publish(throttle_cmd)
        #print(f"Published throttle={throttle_cmd.data}, steering={steering_cmd.data}")
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()