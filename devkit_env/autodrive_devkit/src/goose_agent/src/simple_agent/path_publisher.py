import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import math

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        path_file = ("src/goose_agent/path/cdc-compete/path.csv")

        self.path_pub = self.create_publisher(Path, "/global_path", 10)

        # Load path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        with open(path_file, "r") as f:
            reader = csv.reader(f)
            next(reader)  # skip header

            for row in reader:
                x, y, v, yaw = map(float, row)

                pose = PoseStamped()
                pose.header.frame_id = "world"
                pose.pose.position.x = x
                pose.pose.position.y = y

                # yaw → quaternion
                qz = math.sin(yaw/2)
                qw = math.cos(yaw/2)

                pose.pose.orientation.z = qz
                pose.pose.orientation.w = qw

                self.path_msg.poses.append(pose)

        self.timer = self.create_timer(1, self.timer_cb)

    def timer_cb(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
