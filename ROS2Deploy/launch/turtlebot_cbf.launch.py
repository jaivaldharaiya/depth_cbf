#!/usr/bin/env python3
"""
Launch file for TurtleBot3 CBF Controller.

This launch file starts the CBF-based controller for TurtleBot3
with safe navigation capabilities.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generate launch description for TurtleBot CBF controller."""
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace'
    )
    
    # CBF Controller Node
    cbf_controller_node = Node(
        package='turtle_pkg',
        executable='turtlebot_controller',
        name='turtlebot_cbf_controller',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_namespace_arg,
        cbf_controller_node
    ])