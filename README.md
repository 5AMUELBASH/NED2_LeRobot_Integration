# NED2 LeRobot Integration (ROS2)

This workspace contains two LeRobot plugin packages inside one Python
installation:

- `lerobot_robot_ned2_ros2`: follower-side `Robot` implementation
- `lerobot_teleoperator_ned2_ros2`: leader-side `Teleoperator` implementation

The editable distribution installed from this repo is named
`lerobot_robot_ned2_ros2` so LeRobot's third-party plugin discovery imports it.
That import also loads `lerobot_teleoperator_ned2_ros2`, which registers the
leader plugin from the same root install.

## Prereqs

- ROS2 installed on the system
- `rclpy` importable in the environment
- Niryo ROS2 driver packages for NED2 (for `niryo_ned_ros2_interfaces`)

## Install (editable)

```bash
pip install -e .
```

## Layout

```text
NED2_LeRobot_Integration/
├── pyproject.toml
├── src/
│   ├── lerobot_robot_ned2_ros2/
│   └── lerobot_teleoperator_ned2_ros2/
├── Test_FIles/
└── ned2.urdf
```

## Quick Start

```bash
lerobot-teleoperate \
  --robot.type=ned2_ros2_follower \
  --robot.namespace=/follower \
  --robot.cameras="{}" \
  --teleop.type=ned2_ros2_leader \
  --teleop.namespace=/leader \
  --display_data=false
```

See the config modules in `src/` for full option details.
