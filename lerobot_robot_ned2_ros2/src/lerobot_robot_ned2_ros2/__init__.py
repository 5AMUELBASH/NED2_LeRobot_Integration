"""LeRobot robot plugin for Niryo NED2 over ROS2."""

from .config_ned2_ros2_follower import NED2ROS2FollowerConfig
from .ned2_ros2_follower import NED2ROS2Follower

__all__ = ["NED2ROS2Follower", "NED2ROS2FollowerConfig"]
