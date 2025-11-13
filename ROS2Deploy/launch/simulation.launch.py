#!/usr/bin/env python3
"""
Launch file for TurtleBot3 simulation with CBF controller.

This launch file starts the TurtleBot3 in Gazebo simulation
along with the CBF-based controller.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for TurtleBot simulation with CBF."""
    
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world',
        description='World file name'
    )
    
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='burger',
        description='TurtleBot3 model (burger, waffle, waffle_pi)'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position'
    )
    
    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose')
        }.items()
    )
    
    # CBF Controller Node
    cbf_controller_node = Node(
        package='turtle_pkg',
        executable='turtlebot_controller',
        name='turtlebot_cbf_controller',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        world_arg,
        model_arg,
        x_pose_arg,
        y_pose_arg,
        turtlebot3_gazebo_launch,
        cbf_controller_node
    ])