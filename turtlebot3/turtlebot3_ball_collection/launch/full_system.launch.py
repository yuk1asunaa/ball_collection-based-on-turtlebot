#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import SetParameter

def generate_launch_description():
    # This pipeline depends on RGB-D topics provided by turtlebot3_waffle model.
    os.environ['TURTLEBOT3_MODEL'] = 'waffle'

    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    turtlebot3_ball_collection_dir = get_package_share_directory('turtlebot3_ball_collection')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo world launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'use_sim_time': use_sim_time,
            'gui': LaunchConfiguration('gui'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'spawn_balls': LaunchConfiguration('spawn_balls'),
            'spawn_balls_count': LaunchConfiguration('spawn_balls_count')
        }.items()
    )

    # Ball collection pipeline
    ball_collection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_ball_collection_dir, 'launch', 'ball_collection.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # SLAM + Navigation launch /to alternative pre-built map navigation
    slam_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_ball_collection_dir, 'launch', 'slam_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'launch_rviz': LaunchConfiguration('gui')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_slam'))
    )

    delayed_slam_nav_launch = TimerAction(period=20.0, actions=[slam_nav_launch])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(turtlebot3_gazebo_dir, 'worlds', 'tennis_court.world'),
            description='Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'spawn_balls',
            default_value='true',
            description='Whether to spawn random tennis balls'
        ),
        DeclareLaunchArgument(
            'spawn_balls_count',
            default_value='20',
            description='Number of tennis balls to spawn at startup'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to start Gazebo GUI client'
        ),
        DeclareLaunchArgument(
            'x_pose',
            default_value='0.0',
            description='Robot spawn x position'
        ),
        DeclareLaunchArgument(
            'y_pose',
            default_value='3.0',
            description='Robot spawn y position'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Use SLAM Toolbox for mapping instead of pre-built map'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        SetParameter(name='use_sim_time', value=use_sim_time),
        gazebo_launch,
        ball_collection_launch,
        delayed_slam_nav_launch
    ])
