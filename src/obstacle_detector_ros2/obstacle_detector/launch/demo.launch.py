import os
import sys
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        ComposableNodeContainer(
            name='obstacle_detector_container',
            namespace='robot1',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='obstacle_detector',
                    plugin='obstacle_detector::ObstacleExtractor',
                    name='obstacle_extractor',
                    namespace='robot1',
                    parameters=[{
                        'active': True,
                        'use_scan': True,
                        'use_pcl': False,
                        'use_split_and_merge': False,
                        'circles_from_visibles': True,
                        'discard_converted_segments': True,
                        'transform_coordinates': False,
                        'min_group_points': 2,
                        'max_group_distance': 0.1,
                        'distance_proportion': 0.00628,
                        'max_split_distance': 0.2,
                        'max_merge_separation': 0.1,
                        'max_merge_spread': 0.2,
                        'max_circle_radius': 0.2,
                        'radius_enlargement': 0.1,
                        'frame_id': 'robot1/odom',
                        'use_sim_time': True,
                    }],
                    remappings=[('scan', '/robot1/lidar')],
                ),
                ComposableNode(
                    package='obstacle_detector',
                    plugin='obstacle_detector::PositionEstimator',
                    name='position_estimator',
                    namespace='robot1',
                    parameters=[{
                        'frame_id': 'robot1/odom',
                        'active': True,
                        'use_sim_time': True,
                    }],
                    remappings=[('obstacles', '/robot1/obstacles')],
                )
            ],
        ),
    ]

    return launch.LaunchDescription(nodes)