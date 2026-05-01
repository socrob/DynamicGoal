#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    declare_tiago_gazebo_arg = DeclareLaunchArgument(
        'enable_tiago_gazebo',
        default_value='false',
        description='Enable Tiago Gazebo simulation'
    )

    declare_dynamic_goal_arg = DeclareLaunchArgument(
        'enable_dynamic_goal',
        default_value='false',
        description='Enable dynamic_goal node for people following'
    )

    declare_navigation_node_arg = DeclareLaunchArgument(
        'enable_navigation_node',
        default_value='false',
        description='Enable robot_navigation_api navigation lifecycle node'
    )

    # Get launch configurations
    enable_tiago_gazebo = LaunchConfiguration('enable_tiago_gazebo')
    enable_dynamic_goal = LaunchConfiguration('enable_dynamic_goal')
    enable_navigation_node = LaunchConfiguration('enable_navigation_node')

    # Include Tiago Gazebo launch file
    tiago_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('tiago_gazebo'),
                'launch',
                'tiago_gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'is_public_sim': 'True',
            'world_name': 'pal_office',
            'arm_type': 'no-arm',
            'laser_model': 'hokuyo',
            'navigation': 'True',
            'slam': 'True'
        }.items(),
        condition=IfCondition(enable_tiago_gazebo)
    )

    # Dynamic goal node for people following
    dynamic_goal_node = Node(
        package='dynamic_goal2',
        executable='dynamic_goal2',
        name='dynamic_goal',
        output='screen',
        condition=IfCondition(enable_dynamic_goal)
    )

    # Navigation lifecycle node for the robot_navigation_api package
    navigation_node = Node(
        package='robot_navigation_api',
        executable='navigation_node',
        name='navigation_node',
        output='screen',
        respawn=True,
        condition=IfCondition(enable_navigation_node)
    )

    return LaunchDescription([
        declare_tiago_gazebo_arg,
        declare_dynamic_goal_arg,
        declare_navigation_node_arg,
        tiago_gazebo_launch,
        dynamic_goal_node,
        navigation_node
    ])
