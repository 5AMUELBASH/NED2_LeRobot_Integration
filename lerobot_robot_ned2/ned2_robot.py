import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pyniryo import NiryoRobot
from lerobot.robots import Robot

from lerobot_robot_ned2.ned2_config import Ned2FollowerConfig

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

class Ned2FollowerRobot(Robot):
    name = "ned2_follower"
    config_class = Ned2FollowerConfig

    def __init__(self, config: Ned2FollowerConfig):
        super().__init__(config)
        self.config = config
        self.robot: NiryoRobot | None = None
        self._calibrated = False
        self._last_gripper_open = 1

        self.urdf_path = Path(__file__).parent / "ned2.urdf"

    # ---------- Step 4: calibration/config ----------
    def calibrate(self):
        if self.robot:
            self.robot.calibrate_auto()
            self._calibrated = True

    def configure(self):
        # Optional: apply any settings you always want
        if self.robot:
            self.robot.set_learning_mode(False)
            self.robot.set_arm_max_velocity(80)

    @property
    def is_calibrated(self):
        return self._calibrated

    # ---------- Step 3: connect/disconnect ----------
    def connect(self, calibrate: bool = True):
        self.robot = NiryoRobot(self.config.ip)

        if calibrate:
            # only calibrate if needed, like the reference file
            if self.robot.need_calibration():
                self.robot.calibrate_auto()

        # follower should NOT be in learning mode
        self.robot.set_learning_mode(False)
        self.robot.set_arm_max_velocity(20)
        time.sleep(0.2)
        self._calibrated = True

    def disconnect(self):
        if self.robot:
            self.robot.close_connection()
            self.robot = None

    @property
    def is_connected(self):
        return self.robot is not None

    # ---------- Step 2: feature contracts ----------
    @property
    def observation_features(self):
        # joint_1..joint_6
        obs = {f"joint_{i}.pos": float for i in range(1, 7)}
        obs["gripper.open"] = int  # store last commanded binary state
        return obs

    @property
    def action_features(self):
        act = {f"joint_{i}.pos": float for i in range(1, 7)}
        act["gripper.open"] = int  # 1=open, 0=close
        return act

    # ---------- Step 5: IO loop ----------
    def get_observation(self):
        joints_obj = self.robot.get_joints()
        q = as_joint_list(joints_obj)

        obs = {f"joint_{i}.pos": q[i - 1] for i in range(1, 7)}
        obs["gripper.open"] = int(self._last_gripper_open)
        return obs

    def send_action(self, action: dict[str, Any]):
        q = [float(action[f"joint_{i}.pos"]) for i in range(1, 7)]
        jp = make_joint_position(q)  # guaranteed PyNiryo format

        try:
            self.robot.move(jp)
        except Exception:
            # safest signature across versions: 6 positional floats
            self.robot.move_joints(q[0], q[1], q[2], q[3], q[4], q[5])

        # gripper (keep your current logic)
        g_open = int(action.get("gripper.open", self._last_gripper_open))
        max_torque = getattr(self.config, "gripper_max_torque_percentage", 100)
        hold_torque = getattr(self.config, "gripper_hold_torque_percentage", 30)

        if g_open == 1:
            self.robot.open_gripper(max_torque_percentage=max_torque, hold_torque_percentage=hold_torque)
        else:
            self.robot.close_gripper(max_torque_percentage=max_torque, hold_torque_percentage=hold_torque)

        self._last_gripper_open = g_open
        return action
