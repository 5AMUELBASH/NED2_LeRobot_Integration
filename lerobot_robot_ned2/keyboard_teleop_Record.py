import time
import json
import keyboard
import numpy as np
from pathlib import Path

from ikpy.chain import Chain

from lerobot_robot_ned2.ned2_robot import Ned2Robot
from lerobot_robot_ned2.ned2_config import Ned2Config

from pyniryo import JointsPosition

# =========================
# CONFIG
# =========================
IP = "192.168.8.145"
RATE_HZ = 50
CARTESIAN_STEP = 0.01  # meters per key press
START_RECORD_KEY = "k"
STOP_RECORD_KEY = "l"

DATASET_DIR = Path("datasets")
DATASET_DIR.mkdir(exist_ok=True)

# =========================
# INIT ROBOT
# =========================
config = Ned2Config(ip=IP)
robot = Ned2Robot(config)

print("Connecting to Ned2...")
robot.connect()
print("Connected.")

# =========================
# LOAD URDF + IK CHAIN
# =========================
chain = Chain.from_urdf_file(str(robot.urdf_path))

# Extract joint limits from URDF (only non-fixed joints)
joint_limits = []
for link in chain.links:
    if link.joint_type != 'fixed' and link.bounds is not None:
        joint_limits.append(link.bounds)

# Ned2 has 6 actuated joints
joint_limits = joint_limits[:6]

# =========================
# STATE
# =========================
recording = False
episode = []

print("Cartesian keyboard teleop active")
print("W/S = ±Y | A/D = ±X | Q/E = ±Z")
print("K = start recording | L = stop & save")

last_step_time = time.time()

try:
    while True:
        now = time.time()
        if now - last_step_time < 1.0 / RATE_HZ:
            time.sleep(0.005)
            continue
        last_step_time = now

        # =========================
        # HANDLE RECORDING TOGGLES
        # =========================
        if keyboard.is_pressed(START_RECORD_KEY) and not recording:
            print("▶ Recording started")
            recording = True

        if keyboard.is_pressed(STOP_RECORD_KEY) and recording:
            print("■ Recording stopped")
            break

        # =========================
        # READ CURRENT STATE
        # =========================
        obs = robot.get_observation()
        current_joints = np.array([obs[f"joint_{i}.pos"] for i in range(len(joint_limits))])

        # =========================
        # PAD JOINT VECTOR FOR FIXED LINKS
        # =========================
        full_joint_vector = np.zeros(len(chain.links))
        actuated_idx = 0
        for i, link in enumerate(chain.links):
            if link.joint_type != 'fixed':
                full_joint_vector[i] = current_joints[actuated_idx]
                actuated_idx += 1

        # Forward kinematics
        fk = chain.forward_kinematics(full_joint_vector)
        ee_pos = fk[:3, 3]

        # =========================
        # KEYBOARD → CARTESIAN DELTA
        # =========================
        delta = np.zeros(3)
        if keyboard.is_pressed("w"):
            delta[1] += CARTESIAN_STEP
        if keyboard.is_pressed("s"):
            delta[1] -= CARTESIAN_STEP
        if keyboard.is_pressed("d"):
            delta[0] += CARTESIAN_STEP
        if keyboard.is_pressed("a"):
            delta[0] -= CARTESIAN_STEP
        if keyboard.is_pressed("e"):
            delta[2] += CARTESIAN_STEP
        if keyboard.is_pressed("q"):
            delta[2] -= CARTESIAN_STEP

        if np.linalg.norm(delta) == 0:
            continue

        target_pos = ee_pos + delta

        # =========================
        # IK SOLVE
        # =========================
        ik_solution = chain.inverse_kinematics(
            target_pos,
            initial_position=full_joint_vector
        )

        # Extract only actuated joints from solution
        target_joints = []
        for i, link in enumerate(chain.links):
            if link.joint_type != 'fixed':
                target_joints.append(ik_solution[i])
        target_joints = np.array(target_joints)

        # =========================
        # JOINT LIMIT CLAMPING
        # =========================
        for i, (lower, upper) in enumerate(joint_limits):
            target_joints[i] = np.clip(target_joints[i], lower, upper)

        # =========================
        # SEND ACTION
        # =========================

        # Convert IK solution to Niryo JointsPosition
        robot_position = JointsPosition(*target_joints[:6])
        robot.send_action(robot_position)
        
        # =========================
        # RECORD DATA
        # =========================
        if recording:
            episode.append({
                "timestamp": now,
                "observation": obs,
                "action": action
            })

finally:
    robot.disconnect()


# =========================
# SAVE DATASET
# =========================
if recording and len(episode) > 0:
    outfile = DATASET_DIR / f"cartesian_episode_{int(time.time())}.json"
    with open(outfile, "w") as f:
        json.dump(episode, f, indent=2)

    print(f"✅ Saved {len(episode)} steps to:")
    print(outfile.resolve())
else:
    print("No data recorded.")
