#!/usr/bin/env python3
"""Launch the LLM safety verifier node.

    ros2 launch ros2_llm_safety_verifier verifier_launch.py

Remaps the costmap subscription to Nav2's global costmap topic. The LLM
planner publishes to /proposed_path or /proposed_goal; only proposals
that pass every check appear on /verified_path and /verified_goal.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("ros2_llm_safety_verifier"),
        "config",
        "verifier_params.yaml",
    )
    return LaunchDescription(
        [
            Node(
                package="ros2_llm_safety_verifier",
                executable="verifier_node",
                name="llm_safety_verifier",
                parameters=[params],
                remappings=[("costmap", "/global_costmap/costmap")],
                output="screen",
            )
        ]
    )
