from pyniryo import NiryoRobot
import time

ROBOT_IP = "192.168.8.145"  # change if needed

def main():
    print(f"Connecting to Ned2 at {ROBOT_IP}...")
    robot = NiryoRobot(ROBOT_IP)

    # Give the connection a moment to stabilize
    time.sleep(0.5)

    print("Disabling learning mode...")
    robot.set_learning_mode(True)

    print("Robot is connected and learning mode is OFF.")

    # Optional: verify by trying to read joints
    try:
        joints = list(robot.joints)
        print("Current joints:", joints)
    except Exception as e:
        print("Warning: could not read joints:", e)

    # Keep connection alive briefly
    time.sleep(2)

    print("Disconnecting...")
    robot.close_connection()

if __name__ == "__main__":
    main()
