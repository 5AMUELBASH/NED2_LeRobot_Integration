from lerobot_robot_ned2 import Ned2FollowerConfig, Ned2FollowerRobot
from lerobot_teleoperator_ned2 import Ned2LeaderConfig, Ned2LeaderTeleop

Follower_IP = "192.168.8.145"
Leader_IP = "192.168.8.144"

# Follower robot (the arm that will move)
robot_config = Ned2FollowerConfig(
    ip=Follower_IP,
    id="my_ned2_follower_arm",   # important: used for calibration file naming
    control_rate_hz=20,
)

# Leader teleoperator (the arm you move by hand)
teleop_config = Ned2LeaderConfig(
    ip=Leader_IP,
    id="my_ned2_leader_arm",
    learning_mode=True,         # backdrivable leader
    default_gripper_open=1,     # binary open/close state until you add a toggle
)

robot = Ned2FollowerRobot(robot_config)
teleop_device = Ned2LeaderTeleop(teleop_config)

robot.connect()
teleop_device.connect()

try:
    while True:
        action = teleop_device.get_action()
        robot.send_action(action)
finally:
    teleop_device.disconnect()
    robot.disconnect()
