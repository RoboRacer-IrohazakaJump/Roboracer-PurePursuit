import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan

class DummyScanRepublisher(Node):
    def __init__(self):
        super().__init__('dummy_scan')
        self.sub = self.create_subscription(
            LaserScan,
            '/autodrive/roboracer_1/lidar',
            self.callback,
            10
        )
        self.pub = self.create_publisher(LaserScan, '/dummy/lidar', 10)

    def callback(self, msg):
        msg.header.frame_id = 'dummy_lidar'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DummyScanRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()