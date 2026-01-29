from lerobot_robot_ned2.ned2_robot import Ned2Robot
from lerobot_robot_ned2.ned2_config import Ned2Config

config = Ned2Config()
robot = Ned2Robot(config)

# 1. Required attributes
assert hasattr(robot, "name")
assert hasattr(robot, "config_class")
assert hasattr(robot, "urdf_path")

# 2. Feature schemas
obs_features = robot.observation_features
act_features = robot.action_features

assert isinstance(obs_features, dict)
assert isinstance(act_features, dict)

# 3. Feature types
for k, v in obs_features.items():
    assert isinstance(k, str)
    assert v in (float, int, bool)

for k, v in act_features.items():
    assert isinstance(k, str)
    assert v in (float, int, bool)

print("✅ LeRobot interface compliance PASSED")
