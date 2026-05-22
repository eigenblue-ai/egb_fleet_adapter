#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GDB_PREFIX = 'gdb -batch -ex run -ex "bt full" -ex "info threads" -ex "thread apply all bt full" -ex quit --args'
import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory("egb_fleet_adapter")
    default_config = os.path.join(pkg_dir, "config", "fleet_config.yaml")
    debug = LaunchConfiguration("debug")

    node_kwargs = dict(
        package="egb_fleet_adapter",
        executable="fleet_adapter_node",
        name="egb_fleet_adapter",
        output="screen",
        parameters=[{"server_uri": LaunchConfiguration("server_uri")}],
        arguments=[LaunchConfiguration("config")],
    )

    nodes_group = GroupAction(
        actions=[
            Node(condition=UnlessCondition(debug), **node_kwargs),
            Node(condition=IfCondition(debug), prefix=GDB_PREFIX, **node_kwargs),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="Run fleet adapter under GDB for crash debugging",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Path to fleet configuration YAML file",
            ),
            DeclareLaunchArgument(
                "server_uri",
                default_value="",
                description="URI for RMF API server (websocket), e.g., ws://localhost:8000/_internal",
            ),
            nodes_group,
        ]
    )
