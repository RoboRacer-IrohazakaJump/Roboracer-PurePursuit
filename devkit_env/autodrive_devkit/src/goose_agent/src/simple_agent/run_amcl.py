import rclpy
from rclpy.node import Node
from launch import LaunchService
from launch_ros.actions import Node as LaunchNode

class AMCLRunner(Node):
    def __init__(self):
        super().__init__('amcl_runner')
        self.get_logger().info("Starting AMCL...")

        # Launch AMCL node
        self.amcl_node = LaunchNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=['/home/autodrive_devkit/map/cdc_compete/the_map.yaml']
        )

        # Launch service
        self.ls = LaunchService()
        self.ls.include_launch_description(self.amcl_node)
        self.get_logger().info("AMCL node launched successfully.")

def main(args=None):
    rclpy.init(args=args)
    node = AMCLRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
