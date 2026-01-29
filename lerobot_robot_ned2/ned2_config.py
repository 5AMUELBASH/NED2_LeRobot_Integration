from dataclasses import dataclass
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("ned2_follower")
@dataclass
class Ned2FollowerConfig(RobotConfig):
    ip: str = "192.168.8.145"
    control_rate_hz: int = 20

    # gripper tuning (binary open/close)
    gripper_max_torque_percentage: int = 100
    gripper_hold_torque_percentage: int = 30
