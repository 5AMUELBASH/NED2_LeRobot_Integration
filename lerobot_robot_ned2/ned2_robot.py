import time
from pyniryo import NiryoRobot, JointsPosition
from lerobot.robots import Robot

class Ned2Robot(Robot):
    name = "ned2"

    def __init__(self, config):
        super().__init__(config)
        self.config = config        # <-- store config explicitly
        self.robot = None
        self.dt = 1.0 / self.config.control_rate_hz
        self._calibrated = False


    # ----------------------
    # Required by ABC
    # ----------------------
    def calibrate(self):
        """Perform calibration routine."""
        if self.robot:
            self.robot.calibrate_auto()
            self._calibrated = True

    def configure(self):
        """Optional hardware configuration steps."""
        # No extra configuration needed for Ned2
        pass

    @property
    def is_calibrated(self):
        return self._calibrated

    # ----------------------
    # Existing connect/disconnect
    # ----------------------
    def connect(self):
        self.robot = NiryoRobot(self.config.ip)
        self.robot.calibrate_auto()
        self.robot.set_learning_mode(False)
        self.robot.set_arm_max_velocity(20)
        time.sleep(1)
        self._calibrated = True

    def disconnect(self):
        if self.robot:
            self.robot.close_connection()
            self.robot = None

    @property
    def is_connected(self):
        return self.robot is not None

    # ----------------------
    # Observations / Actions
    # ----------------------
    @property
    def observation_features(self):
        return {f"joint_{i}.pos": float for i in range(6)}

    @property
    def action_features(self):
        return {f"joint_{i}.target_pos": float for i in range(6)}

    def get_observation(self):
        joints = self.robot.get_joints()
        return {f"joint_{i}.pos": joints[i] for i in range(6)}

    def send_action(self, action):
        joints = [action[f"joint_{i}.target_pos"] for i in range(6)]
        self.robot.move(joints)
        time.sleep(self.dt)

    @property
    def name(self):
        return "ned2"
