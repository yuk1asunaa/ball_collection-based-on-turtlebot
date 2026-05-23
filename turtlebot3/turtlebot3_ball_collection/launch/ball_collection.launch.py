#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'waffle')
    robot_model_name = f'turtlebot3_{turtlebot3_model}'

    # Define the path to YOLO model
    yolo_model_path = os.path.join(
        os.path.dirname(get_package_share_directory('turtlebot3_vision')), 
        'yolo11n.pt'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        Node(
            package='turtlebot3_vision',
            executable='yolo_detector_node',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model_path': yolo_model_path,
                'color_topic': '/depth_camera/image_raw',
                'depth_topic': '/depth_camera/depth/image_raw',
                'camera_info_topic': '/depth_camera/camera_info',
                'use_sim_time': use_sim_time,
                'process_every_n_frames': 5,
                'inference_device': 'cpu',
                'inference_imgsz': 640
            }]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='density_map_builder_node',
            name='density_map_builder',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.1,
                'width': 240,
                'height': 240,
                'origin_x': -12.0,
                'origin_y': -12.0,
                'gaussian_sigma': 0.7,
                'time_decay_factor': 0.99,
                'density_increment': 10,
                'max_density': 100,
                'peak_count': 6,
                'decay_period_sec': 1.0,
            }]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'density_weight': 1.0,
                'distance_weight': 0.5,
                'visit_penalty_weight': 0.3,
                'visit_radius': 1.0,
                'spin_angle': 6.283185307179586,
                'collect_duration_sec': 10.0,
                'explore_duration_sec': 120.0,
                'mission_timeout_sec': 300.0,
                'goal_min_separation': 0.5,
            }]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='ball_collector.py',
            name='ball_collector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_states_topic': '/gazebo/model_states',
                'delete_service': '/delete_entity',
                'robot_model_name': 'waffle',
                'robot_name_fallback_substring': 'waffle',
                'ball_name_regex': '(ball|tennis)',
                'collect_distance_m': 0.50,
                'collect_half_fov_rad': 3.14,
                'collect_cooldown_sec': 0.1,
                'check_period_sec': 0.1
            }]
        ),

        Node(
            package='turtlebot3_ball_collection',
            executable='mission_metrics.py',
            name='mission_metrics',
            output='screen',
            parameters=[{
                'model_states_topic': '/gazebo/model_states',
                'robot_model_name': 'waffle',
                'robot_name_fallback_substring': 'waffle',
                'ball_name_regex': '(ball|tennis)',
                'metrics_period_sec': 2.0,
                'finish_zero_count_threshold': 25,
                'output_csv': '/tmp/ball_collection_metrics.csv',
                'use_sim_time': use_sim_time,
            }]
        )
    ])
