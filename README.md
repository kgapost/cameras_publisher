# Cameras Publisher

One ROS 2 node, `publish_cameras.py`, that opens the drone's cameras on the Jetson and publishes them as ROS 2 topics:

- the **stereo pair** (front, CSI, Waveshare/Seeed IMX219-83), used by **obstacle detection**;
- the **belly camera** (down-looking, USB, Arducam B0495), used by **visual odometry**.

It is the only code that touches a camera. Every other module reads these topics.

## Topics

| Topic | Type | Stream |
|---|---|---|
| `/camera/left/image_compressed`, `/camera/right/image_compressed` | `CompressedImage` (JPEG) | stereo |
| `/camera/left/camera_info`, `/camera/right/camera_info` | `CameraInfo` | stereo |
| `/camera/belly/image_compressed` | `CompressedImage` (JPEG) | belly |
| `/camera/belly/camera_info` | `CameraInfo` (calibrated lens, see below) | belly |
| `/tf_static` | `base_link -> camera_left -> camera_right`, `base_link -> camera_belly` | both |

With `--raw` the images go out as `sensor_msgs/Image` on `.../image_raw` instead. **Raw and compressed are different topic names**: publisher and consumers must use the same choice (default: compressed).

Defaults: stereo 640x400 at ~25 Hz, belly 640x400 at ~30 Hz. Belly frames are stamped with the **capture time**, stereo frames with the publish time.

ROS settings: `ROS_DOMAIN_ID=42`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`. They must match on every machine, or the nodes never see each other, with no error.

## Build (on the Jetson)

```bash
git clone https://swarmer.sgx-dev.com/swarmer-group/cameras_publisher.git && cd cameras_publisher
docker build -t cameras-publisher:latest .
```

The image must be built on the Jetson (aarch64). A first build takes about 40 minutes (OpenCV with GStreamer). If the `visual_odometry` or `obstacle_detection` image was already built on that Jetson, Docker reuses its OpenCV layers and the build takes a minute.

## Run

From the repo folder (the folder is mounted, so `logs/` and edits to `config.py` are used without a rebuild):

```bash
# Everything that is plugged in: CSI stereo + USB belly camera
docker run -d --name cameras-publisher --restart unless-stopped --runtime nvidia --network host \
  -v /tmp/argus_socket:/tmp/argus_socket --device /dev/video0 -v "$(pwd)":/workspace cameras-publisher:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 publish_cameras.py --backend gstreamer"

# Stereo only (obstacle detection)
docker run --rm -it --runtime nvidia --network host -v /tmp/argus_socket:/tmp/argus_socket \
  -v "$(pwd)":/workspace cameras-publisher:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 publish_cameras.py --backend gstreamer --no-belly"

# Belly only (visual odometry)
docker run --rm -it --runtime nvidia --network host --device /dev/video0 \
  -v "$(pwd)":/workspace cameras-publisher:latest \
  bash -c "source /opt/ros/jazzy/setup.bash && python3 publish_cameras.py --no-stereo"
```

Check it: `ros2 topic hz /camera/belly/image_compressed`, `docker logs -f cameras-publisher`. Stop: `docker rm -f cameras-publisher`.

Pass the belly camera's `/dev/videoN` with `--device` (the server also tries the other `/dev/video*` nodes it can see). Use `--runtime nvidia`, not `--gpus all`.

**Options:** `--no-stereo`, `--no-belly`, `--backend v4l2|gstreamer` (stereo), `--left/--right /dev/videoN` (v4l2 stereo), `--belly /dev/videoN`, `--belly-backend`, `--compressed/--raw`, `--width/--height/--fps` (v4l2 stereo capture), `--hw-scale` (resize on the Jetson hardware scaler).

## What it does when things go wrong

- At start-up it looks for each camera (belly first, then left and right) and publishes only what it finds. Nothing found: it exits with code 1, so a `--restart` policy retries.
- A camera that sends no frame for 5 s is re-opened (with growing back-off). A watchdog logs `STALLED` / `Recovered` per stream.
- Errors are logged and survived. `docker stop` shuts down cleanly.
- Log: terminal (INFO) and `logs/publish_cameras.log` (DEBUG, rotating 5 MB x 3): devices found, every open/re-open, intrinsics, stalls, errors, and per-stream fps every 5 s.

## Configuration

All settings are in `config.py`. Any of them can be overridden for one run with an environment variable named `OD_<NAME>`, e.g. `-e OD_BELLY_CAMERA_EXPOSURE=30` on `docker run`. The ones you are most likely to set:

| Setting | Default | Meaning |
|---|---|---|
| `STEREO_CAMERA_LEFT/RIGHT_PIPELINE` | `nvarguscamerasrc sensor-id=0/1`, 640x480 @ 25 | GStreamer pipelines of the CSI pair |
| `STEREO_CAMERA_BASELINE_CMS` | `6.0` | Measured distance between the stereo cameras (tf) |
| `FOV_D`, `IMG_W/H_PROCESS` | `83`, `640`/`400` | Stereo CameraInfo and published size |
| `BELLY_CAMERA_DEVICE` | `0` | Belly device tried first |
| `BELLY_CAMERA_FX/FY/CX/CY`, `BELLY_CAMERA_DISTORTION_COEFFS` | `0` / zeros | **Belly lens calibration** (0 = from the 82° FOV) |
| `BELLY_CAMERA_MOUNT_*_DEG`, `BELLY_CAMERA_T*_M` | pitch -90, TZ 0.1 m | Belly mount angles and offset from the IMU (tf) |
| `BELLY_CAMERA_EXPOSURE`, `BELLY_CAMERA_GAIN` | `-1` (auto) | Fixed exposure in 100 µs units (e.g. 30 = 3 ms) |
| `USE_COMPRESSED_IMAGE_TOPICS` | `True` | JPEG vs raw topics |
| `TOPIC_PUBLISHER_TIMER_STEREO/BELLY_CAMERA` | `0.040` / `0.0333` s | Publish periods |

## The belly camera: Arducam B0495

AR0234, 1/2.6" **global shutter**, 1920x1200 (16:10), USB 3.0 (UVC), YUY2 only, M12 lens 3.6 mm, FOV 82° horizontal, fixed focus 2 m to infinity. The server captures 960x600 YUYV at 60 fps and publishes 640x400.

1. **Calibrate the lens** (`cv2.calibrateCamera` with a checkerboard, at 640x400) and put the result in `BELLY_CAMERA_FX/FY/CX/CY` and `BELLY_CAMERA_DISTORTION_COEFFS`. The datasheet values disagree by ~8 %, which becomes an ~8 % scale error in the visual odometry. The VO reads the lens from `/camera/belly/camera_info`, so this is the only place to set it.
2. **Keep the 16:10 shape** (square pixels); the log warns if the capture shape differs.
3. Use a **USB 3** port and cable (USB 2 gives 10 fps). Prefer a short fixed exposure (`BELLY_CAMERA_EXPOSURE`) to limit motion blur.
4. Measure the mount (`BELLY_CAMERA_MOUNT_*`, `BELLY_CAMERA_T*_M`); the default is looking straight down with the image top towards the nose.

## Tools

- `tools/test_stereo_camera.sh`: shows the two CSI cameras in a window from inside the image, no ROS (run `xhost +local:docker` first). Checks the hardware before the full server.
- `python3 write_calibration_yaml.py -o calibration.yaml`: writes the stereo rig calibration (intrinsics, baseline, IMU-to-camera transform) from `config.py`, the same values the server publishes.

## Who uses it

| Module | Needs | Run with |
|---|---|---|
| `obstacle_detection` | stereo pair | `--backend gstreamer --no-belly` |
| `visual_odometry` | belly camera | `--no-stereo` (or no flag if both run) |

In simulation neither module uses this server: `publish_airsim.py` (in each module's repo) publishes the same topics from AirSim.
