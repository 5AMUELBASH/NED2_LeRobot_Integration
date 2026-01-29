import time
from typing import Any

from pyniryo import NiryoRobot
from lerobot.teleoperators.teleoperator import Teleoperator

from lerobot_teleoperator_ned2.ned2_teleop_config import Ned2LeaderConfig

from typing import Sequence
from pyniryo import JointsPosition

def as_joint_list(joints_obj) -> list[float]:
    """Convert PyNiryo joint object (JointsPosition or iterable) into a python list[float] of length 6."""
    if hasattr(joints_obj, "to_list"):
        q = list(joints_obj.to_list())
    else:
        q = list(joints_obj)
    if len(q) != 6:
        raise ValueError(f"Expected 6 joints, got {len(q)}")
    return [float(x) for x in q]

def make_joint_position(q: Sequence[float]) -> JointsPosition:
    """Build a PyNiryo JointsPosition from a sequence of 6 floats."""
    if len(q) != 6:
        raise ValueError(f"Expected 6 joints, got {len(q)}")
    return JointsPosition(*[float(x) for x in q])



class Ned2LeaderTeleop(Teleoperator):
    name = "ned2_leader"
    config_class = Ned2LeaderConfig

    def __init__(self, config: Ned2LeaderConfig):
        super().__init__(config)
        self.config = config
        self.robot: NiryoRobot | None = None
        self._calibrated = False
        self._gripper_open = int(getattr(config, "default_gripper_open", 1))
        self._last_q = None
        self._last_action = None

    # -------- required by Teleoperator ABC --------
    @property
    def action_features(self) -> dict:
        # Must match follower action space
        feats = {f"joint_{i}.pos": float for i in range(1, 7)}
        feats["gripper.open"] = int
        return feats

    @property
    def feedback_features(self) -> dict:
        # No feedback channel for now
        return {}

    def calibrate(self) -> None:
        if self.robot:
            self.robot.calibrate_auto()
            self._calibrated = True

    def configure(self) -> None:
        # Put leader into backdrivable mode
        if self.robot:
            self.robot.set_learning_mode(bool(getattr(self.config, "learning_mode", True)))

    @property
    def is_calibrated(self) -> bool:
        return self._calibrated

    # -------- connection management --------
    @property
    def is_connected(self) -> bool:
        return self.robot is not None

    def connect(self) -> None:
        self.robot = NiryoRobot(self.config.ip)
        self.calibrate()
        self.configure()
        time.sleep(0.2)

    def disconnect(self) -> None:
        if self.robot:
            self.robot.close_connection()
            self.robot = None

    # -------- main I/O --------
    def get_action(self) -> dict[str, Any]:
        joints_obj = self.robot.get_joints()   # likely returns JointsPosition
        q = as_joint_list(joints_obj)          # always becomes list[float] length 6

        action = {f"joint_{i}.pos": q[i - 1] for i in range(1, 7)}
        action["gripper.open"] = int(self._gripper_open)
        return action


    def send_feedback(self, feedback: dict[str, Any]) -> None:
        # No-op for now
        return
