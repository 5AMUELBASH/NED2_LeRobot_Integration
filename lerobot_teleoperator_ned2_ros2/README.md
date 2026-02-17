# LeRobot Teleoperator Plugin: Niryo NED2 (ROS2)

This package provides a LeRobot `Teleoperator` implementation that reads NED2 leader joint states via ROS2.

## Prereqs

- ROS2 installed on the system
- `rclpy` importable in the environment

## Install (editable)

```bash
pip install -e ./lerobot_teleoperator_ned2_ros2
```

## Usage

```bash
lerobot-teleoperate \
  --teleop.type=ned2_ros2_leader \
  --teleop.namespace=/leader \
  --teleop.joint_names='["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]'
```

If `joint_names` is omitted, the teleoperator will use the order from incoming `JointState` messages.

## Config Fields

See `config_ned2_ros2_leader.py` for the full list of options.
