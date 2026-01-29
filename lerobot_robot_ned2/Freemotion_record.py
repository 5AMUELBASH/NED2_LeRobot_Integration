import time
import json
import keyboard
from pathlib import Path

from lerobot_robot_ned2.ned2_robot import Ned2Robot
from lerobot_robot_ned2.ned2_config import Ned2Config

# =====================
# CONFIG
# =====================
IP = "192.168.8.145"
RATE_HZ = 50
START_KEY = "r"
STOP_KEY = "s"
OUTPUT_DIR = Path("datasets")
OUTPUT_DIR.mkdir(exist_ok=True)

# =====================
# INIT ROBOT
# =====================
config = Ned2Config(ip=IP)
robot = Ned2Robot(config)

print("Connecting to Ned2...")
robot.connect()
print("Connected.")

print(f"Press '{START_KEY}' to start recording.")
print(f"Press '{STOP_KEY}' to stop and save.")

episode = []
recording = False

try:
    while True:
        if keyboard.is_pressed(START_KEY) and not recording:
            print("▶ Recording started")
            robot.robot.set_learning_mode(True)  # Freemotion ON
            recording = True
            last_time = time.time()

        if keyboard.is_pressed(STOP_KEY) and recording:
            print("■ Recording stopped")
            robot.robot.set_learning_mode(False)  # Freemotion OFF
            break

        if recording:
            now = time.time()
            if now - last_time >= 1.0 / RATE_HZ:
                obs = robot.get_observation()

                # Action is "next desired position" = current pose
                action = {
                    k.replace(".pos", ".target_pos"): float(v)
                    for k, v in obs.items()
                }

                episode.append({
                    "timestamp": now,
                    "observation": obs,
                    "action": action,
                })

                last_time = now

        time.sleep(0.01)

finally:
    robot.disconnect()

# =====================
# SAVE DATA
# =====================
outfile = OUTPUT_DIR / f"episode_{int(time.time())}.json"
with open(outfile, "w") as f:
    json.dump(episode, f, indent=2)

print(f"✅ Saved episode with {len(episode)} steps to:")
print(outfile.resolve())
