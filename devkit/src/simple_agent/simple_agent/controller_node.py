import rclpy
import numpy as np
import threading
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

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
        timer_period = 0.033  # 30 Hz
        self.create_timer(timer_period, self.control_loop)

        self.lidar_data = None
        self.top_n=90
    
    def lidar_callback(self, msg: LaserScan):
        self.lidar_data = LidarData(msg)

    def compute_direction(self):
        if not self.lidar_data:
            return 0.0, 0.1
        _lidar_data = self.lidar_data
        _ranges = self.lidar_data.ranges
        _top_n = min(self.top_n, len(_ranges))
        _angle_ranges = np.arange(_lidar_data.angle_min, _lidar_data.angle_max, _lidar_data.angle_increment)[10:-10]
        _top_indices = np.argsort(_ranges[10:-10])[-_top_n:]
        #_weights = _ranges[_top_indices]
        _steering = np.sum(_angle_ranges[_top_indices]) / _top_n
        _center_index = len(_ranges) // 2
        _forward_min = _ranges[_center_index-10:_center_index+10].min()
        _throttle = 0.05 if _forward_min > 1.0 else 0.01
        print(_steering, _throttle)
        return _steering, _throttle

    def control_loop(self):
        steering_cmd = Float32()
        throttle_cmd = Float32()

        steering_cmd.data, throttle_cmd.data = self.compute_direction()

        self.steering_pub.publish(steering_cmd)
        self.throttle_pub.publish(throttle_cmd)
        print(f"Published throttle={throttle_cmd.data}, steering={steering_cmd.data}")
    
def main(args=None):
    rclpy.init(args=args)
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()