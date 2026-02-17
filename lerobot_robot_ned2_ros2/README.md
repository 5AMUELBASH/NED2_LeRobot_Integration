# LeRobot Robot Plugin: Niryo NED2 (ROS2)

This package provides a LeRobot `Robot` implementation that controls a Niryo NED2 via ROS2.

## Prereqs

- ROS2 installed on the system
- `rclpy` importable in the environment
- `niryo_ned_ros2_interfaces` available (from the Niryo ROS2 driver)

## Install (editable)

```bash
pip install -e ./lerobot_robot_ned2_ros2
```

## Usage

```bash
lerobot-teleoperate \
  --robot.type=ned2_ros2_follower \
  --robot.namespace=/follower \
  --robot.joint_names='["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]'
```

If `joint_names` is omitted, the robot will use the order from incoming `JointState` messages.

## Config Fields

See `config_ned2_ros2_follower.py` for the full list of options.
