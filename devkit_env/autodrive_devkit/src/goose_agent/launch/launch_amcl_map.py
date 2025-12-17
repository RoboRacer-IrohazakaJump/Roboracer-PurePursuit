import rclpy
from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():    
    # LaunchDescription with lifecycle nodes
    ld = LaunchDescription([
        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='',
            output='screen',
            parameters=['/home/autodrive_devkit/map/cdc_compete/the_map.yaml']
        ),
        LifecycleNode(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace='',
            output='screen',
            parameters=[{'node_names': ['map_server', 'amcl'], 'autostart': True}]
        )
    ])

    return ld