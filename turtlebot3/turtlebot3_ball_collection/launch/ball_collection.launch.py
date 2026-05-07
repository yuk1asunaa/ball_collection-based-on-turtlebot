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
                'width': 100,
                'height': 100,
                'origin_x': -5.0,
                'origin_y': -5.0,
                'gaussian_sigma': 0.5,
                'time_decay_factor': 0.99,
                'navigate_on_peak': True,
                'spin_action_name': '/spin',
                'enable_startup_spin': True,
                'startup_spin_delay_sec': 15.0,
                'lost_target_timeout_sec': 15.0,
                'spin_angle_rad': 6.283185307179586,
                'spin_cooldown_sec': 1.0,
                'enable_lost_target_spin': True,
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
                'collect_distance_m': 0.40,
                'collect_half_fov_rad': 0.70,
                'collect_cooldown_sec': 0.2,
                'check_period_sec': 0.1
            }]
        )
    ])

