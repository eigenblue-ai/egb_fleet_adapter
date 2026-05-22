#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright (c) 2026 The EgbFleetAdapter Authors
# Launch file for C++ egb_fleet_adapter

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

GDB_PREFIX = 'gdb -batch -ex run -ex "bt full" -ex "info threads" -ex "thread apply all bt full" -ex quit --args'
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    debug = LaunchConfiguration("debug")

    # Read domain IDs from environment variables
    # FOREIGN_DOMAIN_ID: foreign domain (publishes /clock)
    # ROS_DOMAIN_ID: OpenRMF/Fleet adapter domain (consumes /clock)
    foreign_domain_id = os.environ.get("FOREIGN_DOMAIN_ID")
    ros_domain_id = os.environ.get("ROS_DOMAIN_ID")

    if not foreign_domain_id or not ros_domain_id:
        raise RuntimeError(
            f"Missing required environment variables: "
            f"FOREIGN_DOMAIN_ID={foreign_domain_id}, ROS_DOMAIN_ID={ros_domain_id}. "
            f"These should be set by the Helm chart init containers."
        )

    # Generate dynamic domain bridge config
    # Bridge /clock FROM foreign domain TO OpenRMF domain
    domain_bridge_config_data = {
        "from_domain": int(foreign_domain_id),
        "to_domain": int(ros_domain_id),
        "topics": {"/clock": {"type": "rosgraph_msgs/msg/Clock"}},
    }

    # Write to temporary file
    temp_config = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="domain_bridge_config_"
    )
    yaml.dump(domain_bridge_config_data, temp_config, default_flow_style=False)
    temp_config.close()
    domain_bridge_config = temp_config.name

    print(
        f"[fleet_spawner_with_bridge] Generated domain bridge config: {domain_bridge_config}"
    )
    print(
        f"[fleet_spawner_with_bridge] Bridging /clock from foreign domain {foreign_domain_id} to OpenRMF domain {ros_domain_id}"
    )

    # Fleet spawner node — passes debug flag so it can launch fleet adapter under GDB
    fleet_spawner_node = Node(
        package="egb_fleet_spawner",
        executable="egb_fleet_spawner",
        name="egb_fleet_spawner",
        output="screen",
        parameters=[
            {
                "debug": debug,
            }
        ],
    )

    # Fleet service bridge (C++ executable) - bridges services only
    # Bridges fleet adapter services FROM domain 45 TO domain 41
    bridge_kwargs = dict(
        package="egb_fleet_spawner",
        executable="fleet_service_bridge",
        name="fleet_service_bridge",
        output="screen",
        arguments=["--ros-args", "--log-level", "rmw_cyclonedds_cpp:=error"],
    )
    fleet_service_bridge_group = GroupAction(
        actions=[
            Node(condition=UnlessCondition(debug), **bridge_kwargs),
            Node(condition=IfCondition(debug), prefix=GDB_PREFIX, **bridge_kwargs),
        ]
    )

    # Domain bridge for topics (YAML config) - bridges topics only
    # Bridges /clock topic FROM domain 41 TO domain 45
    clock_topic_bridge_node = Node(
        package="domain_bridge",
        executable="domain_bridge",
        name="clock_topic_bridge",
        output="screen",
        arguments=[
            domain_bridge_config,
            "--ros-args",
            "--log-level",
            "rmw_cyclonedds_cpp:=error",
        ],
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Run C++ nodes under GDB for crash debugging",
    )

    return LaunchDescription(
        [
            debug_arg,
            fleet_spawner_node,
            fleet_service_bridge_group,
            clock_topic_bridge_node,
        ]
    )
