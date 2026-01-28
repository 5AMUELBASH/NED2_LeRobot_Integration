import lerobot_robot_ned2
from lerobot_robot_ned2.ned2_config import Ned2Config
from lerobot_robot_ned2.ned2_robot import Ned2Robot

config = Ned2Config(ip="169.254.200.200")
robot = Ned2Robot(config)
robot.connect()

obs = robot.get_observation()
print("Joint observation:", obs)

robot.disconnect()
