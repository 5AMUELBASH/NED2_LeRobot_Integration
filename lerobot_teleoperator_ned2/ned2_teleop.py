from typing import Any
from pyniryo import NiryoRobot

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot_teleoperator_ned2.ned2_teleop_config import Ned2LeaderConfig


class Ned2LeaderTeleop(Teleoperator):
    name = "ned2_leader"
    config_class = Ned2LeaderConfig

    def __init__(self, config: Ned2LeaderConfig):
        super().__init__(config)
        self.config = config
        self.robot: NiryoRobot | None = None
        self._gripper_open = int(config.default_gripper_open)

    @property
    def is_connected(self) -> bool:
        return self.robot is not None

    def connect(self) -> None:
        self.robot = NiryoRobot(self.config.ip)
        self.robot.connect()
        self.robot.calibrate_auto()

        # leader should be backdrivable
        self.robot.set_learning_mode(bool(self.config.learning_mode))

    def disconnect(self) -> None:
        if self.robot:
            self.robot.close_connection()
            self.robot = None

    def get_action(self) -> dict[str, Any]:
        j = self.robot.get_joints()
        action = {f"joint_{i}.pos": float(j[i - 1]) for i in range(1, 7)}
        action["gripper.open"] = int(self._gripper_open)
        return action

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # optional
        return
