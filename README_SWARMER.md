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
At start-up it looks for each camera and publishes only what it finds (stereo, belly, or both). A CSI camera that does not open within 10 s counts as missing. If it finds none it exits and Docker restarts it, so cameras plugged in later are picked up. A camera that stops sending frames is re-opened automatically.

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
