# cameras_publisher (SWARMER module)

Publishes the drone's cameras on ROS 2: the front **stereo pair** (CSI, used by `obstacle_detection`) and the **belly camera** (USB, used by `visual_odometry`). It is the only module that opens the cameras.

## Requirements
- Jetson with JetPack 6 (L4T r36.x), Docker with the NVIDIA container runtime (JetPack default) and Docker Compose.
- `nvargus-daemon` running on the host (JetPack default), for the CSI stereo pair.
- Belly camera (Arducam B0495) on a **USB 3** port.
- **One** instance per drone: it serves every module.

## Build
```bash
docker compose build          # on the Jetson, a few minutes (image: cameras-publisher:latest)
```

## Deploy
```bash
docker compose up -d          # start (also after every reboot: restart: unless-stopped)
docker compose logs -f        # log; also ./logs/publish_cameras.log
docker compose down           # stop
```
## Cameras
The setting that controls each point is in parentheses (`config.py`, changeable as `OD_<NAME>` in `docker-compose.yml`).

**Belly: Arducam B0495** (AR0234, 2.3 MP, global shutter, USB 3.0 UVC). Opened with V4L2 (`BELLY_CAMERA_BACKEND`). Captured at 960x600 YUYV, 60 fps (`BELLY_CAMERA_CAPTURE_WIDTH/HEIGHT`, `BELLY_CAMERA_FOURCC`, `BELLY_CAMERA_FPS`); published at 640x400 (`BELLY_CAMERA_WIDTH/HEIGHT`), **30 Hz** (`TOPIC_PUBLISHER_TIMER_BELLY_CAMERA` = 0.0333 s).

**Front stereo: Waveshare IMX219-83 Stereo Camera** (two 8 MP IMX219, 83° FOV (`FOV_D`), 60 mm baseline (`STEREO_CAMERA_BASELINE_CMS`)), on the Jetson's two **CSI** ports. A CSI sensor gives raw Bayer data (`RG10`) that plain V4L2/OpenCV cannot use; only NVIDIA's `nvarguscamerasrc` (Jetson ISP) turns it into images, and it is a **GStreamer** element. That is why the stereo pair is opened through GStreamer pipelines (`STEREO_CAMERA_BACKEND` = `gstreamer`) and why OpenCV must be built with GStreamer support: the image uses Ubuntu's OpenCV package, which has it. Left = `sensor-id=0`, right = `sensor-id=1` (`STEREO_CAMERA_LEFT/RIGHT_PIPELINE`), captured at 640x480, 25 fps; published at 640x400 (`IMG_W/H_PROCESS`), **25 Hz** (`TOPIC_PUBLISHER_TIMER_STEREO_CAMERA` = 0.040 s).

**Search order at start-up** (each stream can be switched off: `BELLY_CAMERA_ENABLED`, `STEREO_CAMERA_ENABLED`, or `--no-belly` / `--no-stereo`):
1. **Belly** first: `/dev/video0` (`BELLY_CAMERA_DEVICE`), then every other `/dev/video*` in number order. Skipped: the CSI sensor nodes and nodes that open but deliver no frame (e.g. the camera's UVC metadata node). The first node that delivers frames is the belly camera.
2. **Left** stereo camera, then 3. **Right**, each on its own (never the belly's node). A camera whose pipeline does not open within 10 s counts as missing (`CAMERA_OPEN_TIMEOUT_S`).

**If a camera is not found or fails:**
- **None found:** error in the log, exit code 1, and Docker restarts the container (`restart: unless-stopped`), which searches again.
- **Some found:** it publishes those (belly only, stereo only, left only, ...). The missing ones are **not** searched again while it runs: after plugging one in, `docker compose restart`.
- **A camera stops sending frames:** the log shows `STALLED` after 3 s; after 5 s without frames it is re-opened (`BELLY_CAMERA_REOPEN_AFTER_S`, `STEREO_CAMERA_REOPEN_AFTER_S`), then retried with a growing pause (up to 30 s). The belly camera is searched again over all `/dev/video*`, since its node can change. `Recovered` is logged when frames return.

## Topics
Publishes only; subscribes to nothing. QoS: reliable, volatile, depth 10.

| Topic | Type | Rate |
|---|---|---|
| `/camera/left/image_compressed`, `/camera/right/image_compressed` | `sensor_msgs/CompressedImage` (JPEG, 640x400) | ~25 Hz |
| `/camera/left/camera_info`, `/camera/right/camera_info` | `sensor_msgs/CameraInfo` | with the images |
| `/camera/belly/image_compressed` | `sensor_msgs/CompressedImage` (JPEG, 640x400) | ~30 Hz |
| `/camera/belly/camera_info` | `sensor_msgs/CameraInfo` (calibrated lens) | with the images |
| `/tf_static` | `base_link -> camera_left -> camera_right`, `base_link -> camera_belly` | once (transient local) |

- Frame ids: `camera_left`, `camera_right`, `camera_belly` (body FRD axes).
- Belly images are stamped with the **capture time**, stereo images with the publish time.
- With `OD_USE_COMPRESSED_IMAGE_TOPICS: "0"` the images go out raw (`sensor_msgs/Image`) on `.../image_raw`. All consumers must use the same setting.

## Settings
- `ROS_DOMAIN_ID` (default **42**) must be the same for every SWARMER module and for PX4's uXRCE-DDS agent: `ROS_DOMAIN_ID=0 docker compose up -d`. The RMW is Cyclone DDS.
- Any setting of `config.py` can be changed in `docker-compose.yml` as `OD_<NAME>` (no rebuild).
- **Belly lens calibration**: set `OD_BELLY_CAMERA_FX/FY/CX/CY` and `OD_BELLY_CAMERA_DISTORTION_COEFFS` once per camera (checkerboard at 640x400). Uncalibrated, the visual odometry has a scale error of about 8 %.
- Measure and set the mounts: `OD_BELLY_CAMERA_MOUNT_*_DEG`, `OD_BELLY_CAMERA_T*_M` (offset from the IMU) and `OD_STEREO_CAMERA_BASELINE_CMS` (6 cm).
- Fixed belly exposure (less motion blur): `OD_BELLY_CAMERA_EXPOSURE: "30"` (100 µs units; default auto).

## Check
```bash
ros2 topic hz /camera/belly/image_compressed     # ~30 Hz
ros2 topic hz /camera/left/image_compressed      # ~25 Hz
```
Raw check of the CSI cameras, no ROS: `xhost +local:docker && tools/test_stereo_camera.sh`.
