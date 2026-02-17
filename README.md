# NED2 LeRobot Integration (ROS2)

This workspace contains two LeRobot plugins:

- `lerobot_robot_ned2_ros2`: follower-side Robot implementation
- `lerobot_teleoperator_ned2_ros2`: leader-side Teleoperator implementation

## Prereqs

- ROS2 installed on the system
- `rclpy` importable in the environment
- Niryo ROS2 driver packages for NED2 (for `niryo_ned_ros2_interfaces`)

## Install (editable)

```bash
pip install -e ./lerobot_robot_ned2_ros2
pip install -e ./lerobot_teleoperator_ned2_ros2
```

## Quick Start

```bash
lerobot-teleoperate \
  --robot.type=ned2_ros2_follower \
  --teleop.type=ned2_ros2_leader
```

See each package README for configuration details.
