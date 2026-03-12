import cv2
import time

CAMERA_ID = 0 # change this

def main():
    # Force Linux V4L2 backend (often fixes “window but no frames”)
    cap = cv2.VideoCapture(CAMERA_ID, cv2.CAP_V4L2)

    if not cap.isOpened():
        print(f"Failed to open camera {CAMERA_ID}")
        return

    # Try a common format that many webcams support
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)

    # Warm-up: some cameras need a few frames before they output valid data
    for _ in range(10):
        cap.read()
        time.sleep(0.02)

    print("Camera started. Press 'q' to quit.")

    while True:
        ret, raw = cap.read()
        frame = raw

        if not ret or frame is None:
            print("No frame (ret=False or frame=None).")
            time.sleep(0.05)
            continue

        # Debug: prove we got real pixels
        h, w = frame.shape[:2]
        print(f"\rFrame: {w}x{h}  ", end="")

        cv2.imshow("Camera Feed", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
