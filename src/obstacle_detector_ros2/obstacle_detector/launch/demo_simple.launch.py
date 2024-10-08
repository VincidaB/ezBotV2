import os
import sys
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

# scans_merger
# obstacle_extractor
# obstacle_tracker
# obstacle_publisher

def generate_launch_description():
    nodes = [
        ComposableNodeContainer(
            name='obstacle_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='urg_node',
                    plugin='urg_node::UrgNode',
                    name='urg_node_driver',
                    parameters=[
                        {'ip_address': '192.168.10.10'}
                    ],
                ),
                ComposableNode(
                    package='obstacle_detector',
                    plugin='obstacle_detector::ObstacleExtractor',
                    name='obstacle_extractor',
                    parameters=[
                        {'active': True},
                        {'use_scan': True},
                        {'use_pcl': False},
                        {'use_split_and_merge': True},
                        {'circles_from_visibles': True},
                        {'discard_converted_segments': True},
                        {'transform_coordinates': True},
                        {'min_group_points': 5},
                        {'max_group_distance': 0.1},
                        {'distance_proportion': 0.00436},
                        {'max_split_distance': 0.2},
                        {'max_merge_separation': 0.2},
                        {'max_merge_spread': 0.2},
                        {'max_circle_radius': 0.6},
                        {'radius_enlargement': 0.3},
                        {'frame_id': 'laser'},
                    ],
                ),
            ],
        ),
        # Node(
        #     package='obstacle_detector',
        #     executable='obstacle_extractor_node',
        #     name='obstacle_extractor_node',
        #     parameters=[
        #         {'active': True},
        #         {'use_scan': True},
        #         {'use_pcl': False},
        #         {'use_split_and_merge': True},
        #         {'circles_from_visibles': True},
        #         {'discard_converted_segments': True},
        #         {'transform_coordinates': True},
        #         {'min_group_points': 5},
        #         {'max_group_distance': 0.1},
        #         {'distance_proportion': 0.00628},
        #         {'max_split_distance': 0.2},
        #         {'max_merge_separation': 0.2},
        #         {'max_merge_spread': 0.2},
        #         {'max_circle_radius': 0.6},
        #         {'radius_enlargement': 0.3},
        #         {'frame_id': 'map'},
        #     ],
        # ),

        # Node(
        #     package='urg_node',
        #     executable='urg_node_driver',
        #     name='urg_node_driver',
        #     parameters=[
        #         {'ip_address': '192.168.10.10'}
        #     ],
        # ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_scanner_base',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'scanner_base'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='scanner_base_to_robot',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'scanner_base', 'robot'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='robot_to_laser',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'robot', 'laser'],
        )
    ]

    return launch.LaunchDescription(nodes)
