#!/usr/bin/env python

# Copyright 2026
# Licensed under the Apache License, Version 2.0

from dataclasses import dataclass, field
from typing import TypeAlias

from lerobot.robots.config import RobotConfig


@RobotConfig.register_subclass("ned2_ros2_follower")
@dataclass
class NED2ROS2FollowerConfig(RobotConfig):
    """Configuration for the Niryo NED2 follower controlled via ROS2."""

    namespace: str = "/follower"

    # Topics / actions (relative to namespace if not absolute)
    joint_states_topic: str = "joint_states"
    action_name: str = "niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory"

    # Explicit joint order from the URDF
    joint_names: list[str] = field(
        default_factory=lambda: [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]
    )

    # Trajectory timing (seconds)
    point_time: float = 0.3

    # Gripper services / topics
    update_tool_service: str = "niryo_robot_tools_commander/update_tool"
    open_gripper_service: str = "niryo_robot/tools/open_gripper"
    close_gripper_service: str = "niryo_robot/tools/close_gripper"
    tool_motor_topic: str = "niryo_robot_hardware/tools/motor"
    # Gripper behavior
    gripper_key: str = "gripper"
    gripper_open_value: float = 1.0
    gripper_close_value: float = 0.0
    gripper_toggle_threshold: float = 0.5

    tool_id: int = 13
    gripper_open_pos: int = 2900
    gripper_close_pos: int = 1900
    gripper_speed: int = 900
    gripper_hold_torque: int = 150
    gripper_max_torque: int = 150

    update_tool_on_connect: bool = True
    update_tool_each_toggle: bool = False

    # Startup behavior
    startup_timeout_s: float = 10.0
    wait_for_joint_states: bool = True

    # Advanced: set to True only if this plugin owns the ROS2 context
    shutdown_rclpy_on_disconnect: bool = False


NED2ROS2FollowerConfigAlias: TypeAlias = NED2ROS2FollowerConfig
