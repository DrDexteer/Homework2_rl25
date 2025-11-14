from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    params_file    = LaunchConfiguration('params_file')
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ros2_kdl_package'), 'config', 'params.yaml'
            ]),
            description='YAML file with the params.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'marker_id',
            default_value="201",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'marker_size',
            default_value="0.1",
        )
    )

    ros2_kdl_node = Node(
        package="ros2_kdl_package",
        executable="ros2_kdl_node",
        name="ros2_kdl_node",
        output="screen",
        parameters=[
            params_file,     # loads YAML file
            
        ],
    )

    aruco_single = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('aruco_ros'), 'launch', 'single.launch.py'])
        ),
        launch_arguments={
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size': LaunchConfiguration('marker_size'),
        }.items(),

    )
    
    nodes_to_start = []    

    nodes_to_start = [
        ros2_kdl_node,        
        aruco_single,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start) 