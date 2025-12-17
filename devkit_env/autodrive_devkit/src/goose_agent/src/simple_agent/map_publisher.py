#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from nav2_map_server.map_server import MapServer
import os

class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')

        # Map file path
        self.map_file = self.declare_parameter(
            'map_file', 
            '/home/autodrive_devkit/map/cdc_compete/the_map.yaml'
        ).get_parameter_value().string_value

        # Load the map server
        self.map_server = self.create_node_map_server()

    def create_node_map_server(self):
        """
        Launch the nav2 MapServer programmatically
        """
        from nav2_map_server.map_server import MapServer
        map_server_node = MapServer(
            self.get_node_logger(),
            self.get_node_parameters(),
            self.get_node_name()
        )
        return map_server_node

    def get_node_logger(self):
        return self.get_logger()

    def get_node_parameters(self):
        return {'yaml_filename': self.map_file, 'frame_id': 'map'}

    def get_node_name(self):
        return self.get_name()


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()