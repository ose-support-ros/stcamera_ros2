#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # adapt if needed
    debug = False
    respawn = False

    default_config_file = os.path.join(
        get_package_share_directory('stcamera_launcher'),
        'config',
        'default.yaml'
    )

    # launch configuration variables
    config_file = LaunchConfiguration('config_file')
    namespace_value = LaunchConfiguration('namespace_value')
    node_name = LaunchConfiguration('node_name')

    # launch arguments
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Camera parameters structured in a .yaml file.'
    )
    declare_namespace_value_cmd = DeclareLaunchArgument(
        'namespace_value',
        default_value='stcamera_launcher',
        description='Namespace.'
    )
    declare_node_name_cmd = DeclareLaunchArgument(
        'node_name',
        default_value='stcameras',
        description='Name of the node.'
    )

    # log format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # see https://navigation.ros.org/tutorials/docs/get_backtrace.html
    if (debug == True):
        launch_prefix = ['xterm -e gdb -ex run --args']
    else:
        launch_prefix = ''

    # node
    stcamera_node = Node(
        package='stcamera_launcher',        
        namespace=namespace_value,
        executable='stcamera_launcher',
        name=node_name,
        output='screen',
        respawn=respawn,
        emulate_tty=True,
        prefix=launch_prefix,
        parameters=[
            config_file
        ]
    )

    # Define LaunchDescription variable and return it
    ld = LaunchDescription()

    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_namespace_value_cmd)
    ld.add_action(declare_node_name_cmd)

    ld.add_action(stcamera_node)

    return ld
