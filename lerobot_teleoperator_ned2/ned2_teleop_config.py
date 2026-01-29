from dataclasses import dataclass
from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("ned2_leader")
@dataclass
class Ned2LeaderConfig(TeleoperatorConfig):
    ip: str = "192.168.8.145"

    # leader backdrive mode
    learning_mode: bool = True

    # how to set gripper if you do not have a leader gripper sensor
    default_gripper_open: int = 1
