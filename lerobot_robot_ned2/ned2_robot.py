import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pyniryo import NiryoRobot
from lerobot.robots import Robot

from lerobot_robot_ned2.ned2_config import Ned2FollowerConfig


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
            self.robot.set_arm_max_velocity(20)

    @property
    def is_calibrated(self):
        return self._calibrated

    # ---------- Step 3: connect/disconnect ----------
    def connect(self):
        self.robot = NiryoRobot(self.config.ip)
        self.robot.connect()
        self.calibrate()
        self.configure()
        time.sleep(0.2)

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
        joints = self.robot.get_joints()
        obs = {f"joint_{i}.pos": float(joints[i - 1]) for i in range(1, 7)}
        obs["gripper.open"] = int(self._last_gripper_open)
        return obs

    def send_action(self, action: dict[str, Any]):
        # 1) joints
        q = [float(action[f"joint_{i}.pos"]) for i in range(1, 7)]

        # Prefer move(JointsPosition) if you want, but move_joints also works
        try:
            from pyniryo import JointsPosition
            self.robot.move(JointsPosition(*q))
        except Exception:
            self.robot.move_joints(q)

        # 2) binary gripper
        g_open = int(action.get("gripper.open", self._last_gripper_open))
        if g_open == 1:
            self.robot.open_gripper(
                max_torque_percentage=self.config.gripper_max_torque_percentage,
                hold_torque_percentage=self.config.gripper_hold_torque_percentage,
            )
        else:
            self.robot.close_gripper(
                max_torque_percentage=self.config.gripper_max_torque_percentage,
                hold_torque_percentage=self.config.gripper_hold_torque_percentage,
            )
        self._last_gripper_open = g_open

        return action
