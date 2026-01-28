from dataclasses import dataclass
from lerobot.robots import RobotConfig

@RobotConfig.register_subclass("ned2")
@dataclass
class Ned2Config(RobotConfig):
    ip: str = "169.254.200.200"
    control_rate_hz: int = 20
