import cv2
import numpy as np

CAPTURE_WIDTH = 1280
CAPTURE_HEIGHT = 720
DISPLAY_WIDTH = 320
DISPLAY_HEIGHT = 180
FRAMERATE = 60
FLIP_METHOD = 0  # 0=none, 2=180 rotate, 1/3=90 rotate, etc.

# --- Locked exposure/gain/whitebalance for stereo consistency ---
# Fixing these prevents each sensor's AE/AWB from drifting independently,
# which otherwise injects brightness mismatches into stereo matching.
# Tune EXPOSURE_TIME_NS and gain values for your lighting conditions.
EXPOSURE_TIME_NS = "13000000 13000000"   # fixed at ~13ms (adjust to your lighting)
GAIN_RANGE = "1 1"                        # fixed analog gain (sensor min/max for IMX219 is roughly 1-10.6)
ISP_DIGITAL_GAIN_RANGE = "1 1"            # fixed ISP digital gain, disables auto brightness boost
WBMODE = 1                                 # 1 = fixed/off preset (0=off in some builds; see note below)

def gstreamer_pipeline(sensor_id):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} "
        f"aelock=true awblock=true "
        f"exposuretimerange=\"{EXPOSURE_TIME_NS}\" "
        f"gainrange=\"{GAIN_RANGE}\" "
        f"ispdigitalgainrange=\"{ISP_DIGITAL_GAIN_RANGE}\" "
        f"wbmode={WBMODE} ! "
        f"video/x-raw(memory:NVMM), width={CAPTURE_WIDTH}, height={CAPTURE_HEIGHT}, "
        f"framerate={FRAMERATE}/1 ! "
        f"nvvidconv flip-method={FLIP_METHOD} ! "
        f"video/x-raw, width={DISPLAY_WIDTH}, height={DISPLAY_HEIGHT}, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=1"
    )

def main():
    cap0 = cv2.VideoCapture(gstreamer_pipeline(0), cv2.CAP_GSTREAMER)
    cap1 = cv2.VideoCapture(gstreamer_pipeline(1), cv2.CAP_GSTREAMER)

    if not cap0.isOpened():
        print("Failed to open camera 0. Check sensor-id, ribbon cable, and that no other process is using it.")
        return
    if not cap1.isOpened():
        print("Failed to open camera 1. Check sensor-id, ribbon cable, and that no other process is using it.")
        cap0.release()
        return

    blank_frame = np.zeros((DISPLAY_HEIGHT, DISPLAY_WIDTH, 3), dtype=np.uint8)

    print("Streaming cameras 0 and 1 side by side. Press 'q' to quit.")
    try:
        while True:
            ret0, frame0 = cap0.read()
            ret1, frame1 = cap1.read()

            if not ret0:
                print("Failed to grab frame from camera 0.")
                frame0 = blank_frame
            if not ret1:
                print("Failed to grab frame from camera 1.")
                frame1 = blank_frame

            cv2.imshow("Stereo Cameras (0 | 1)", cv2.hconcat([frame0, frame1]))

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap0.release()
        cap1.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
