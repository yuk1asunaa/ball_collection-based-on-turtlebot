#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='false')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    # World file to load in Gazebo (can be overridden at launch time)
    default_world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'tennis_court.world'
    )

    world = LaunchConfiguration('world', default=default_world)

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Gazebo world file to load'
    )

    declare_spawn_balls_cmd = DeclareLaunchArgument(
        'spawn_balls', default_value='true',
        description='Whether to spawn random tennis balls'
    )

    declare_spawn_balls_count_cmd = DeclareLaunchArgument(
        'spawn_balls_count', default_value='20',
        description='Number of random tennis balls to spawn'
    )

    declare_gui_cmd = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Whether to start Gazebo client GUI (gzclient)'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world
        }.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        condition=IfCondition(gui)
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Optional: spawn random tennis balls after Gazebo starts
    spawn_balls_script = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'scripts',
        'spawn_random_tennis_balls.py'
    )

    spawn_random_balls_cmd = ExecuteProcess(
        cmd=['python3', spawn_balls_script, '--count', LaunchConfiguration('spawn_balls_count')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('spawn_balls', default='true'))
    )

    ld = LaunchDescription()

    # Declare launch arguments before adding actions
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_spawn_balls_cmd)
    ld.add_action(declare_spawn_balls_count_cmd)
    ld.add_action(declare_gui_cmd)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(spawn_random_balls_cmd)

    return ld
