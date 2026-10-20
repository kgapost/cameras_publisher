# cameras_publisher (SWARMER module)

Publishes the drone's cameras on ROS 2: the **belly camera** (USB, used by `visual_odometry`) and, optionally, the front **stereo pair** (CSI, used by `obstacle_detection`; **off by default**: switch it on with `CP_STEREO_CAMERA_ENABLED=1` in `.env` or `--stereo`). It is the only module that opens the cameras.

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
Edit `.env` next to `docker-compose.yml` before starting. Compose passes its
settings into the container. Defaults select belly only; for stereo only set
`CP_BELLY_CAMERA_ENABLED=0` and `CP_STEREO_CAMERA_ENABLED=1`. Leave
`CP_CAPTURE_API_MODE=auto` to map the capture services accordingly. Apply `.env`
changes with `docker compose up -d`; no rebuild is required for settings alone.

```bash
docker compose up -d          # start (also after every reboot: restart: unless-stopped)
docker compose up -d --build  # after a git pull: the code is in the image, without --build the old code runs
docker compose logs -f        # live log (see Logs below)
docker compose down           # stop
```
## Cameras
The setting that controls each point is in parentheses (`config.py`, changeable as `CP_<NAME>` in `docker-compose.yml`).

**Belly: Arducam B0495** (AR0234, 2.3 MP, global shutter, USB 3.0 UVC). Opened with V4L2 (`BELLY_CAMERA_BACKEND`). Captured at 960x600 YUYV, 60 fps (`BELLY_CAMERA_CAPTURE_WIDTH/HEIGHT`, `BELLY_CAMERA_FOURCC`, `BELLY_CAMERA_FPS`); published at 640x400 (`BELLY_CAMERA_WIDTH/HEIGHT`), **30 Hz** (`TOPIC_PUBLISHER_TIMER_BELLY_CAMERA` = 0.0333 s).

**Front stereo: Waveshare IMX219-83 Stereo Camera** (two 8 MP IMX219, 83° FOV (`FOV_D`), 60 mm baseline (`STEREO_CAMERA_BASELINE_CMS`)), on the Jetson's two **CSI** ports. A CSI sensor gives raw Bayer data (`RG10`) that plain V4L2/OpenCV cannot use; only NVIDIA's `nvarguscamerasrc` (Jetson ISP) turns it into images, and it is a **GStreamer** element. That is why the stereo pair is opened through GStreamer pipelines (`STEREO_CAMERA_BACKEND` = `gstreamer`) and why OpenCV must be built with GStreamer support: the image uses Ubuntu's OpenCV package, which has it. Left = `sensor-id=0`, right = `sensor-id=1` (`STEREO_CAMERA_LEFT/RIGHT_PIPELINE`), captured at 640x480, 25 fps; published at 640x400 (`IMG_W/H_PROCESS`), **25 Hz** (`TOPIC_PUBLISHER_TIMER_STEREO_CAMERA` = 0.040 s).

**Search order at start-up** (each stream is switched with `BELLY_CAMERA_ENABLED` / `STEREO_CAMERA_ENABLED` or `--no-belly` / `--stereo`; the stereo pair is off by default and is not searched):
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
- With `CP_USE_COMPRESSED_IMAGE_TOPICS: "0"` the images go out raw (`sensor_msgs/Image`) on `.../image_raw`. All consumers must use the same setting.

## Capture API (save images and videos)

ROS 2 and HTTP expose the same seven commands. The endpoint selects the camera;
every request contains only `name`. Use a distinct name per module or recording
session, or pass `default` explicitly for a default capture.

| Command suffix | Belly mode | Stereo mode |
|---|---|---|
| `health` | Publish/return health JSON | Publish/return health JSON |
| `image1` | Save belly JPEG | Save left JPEG |
| `image2` | Warn: no camera | Save right JPEG |
| `video1_start` | Start named belly MP4 | Start named left MP4 |
| `video2_start` | Warn: no camera | Start named right MP4 |
| `video1_stop` | Finalize named belly MP4 | Finalize named left MP4 |
| `video2_stop` | Warn: no camera | Finalize named right MP4 |

ROS service names are `/camera/capture/<suffix>`; HTTP paths are `/api/<suffix>`.
The ROS namespace is configurable with `CP_CAPTURE_API_NAMESPACE`.
`CP_CAPTURE_API_MODE=auto` selects stereo mapping when `STEREO_CAMERA_ENABLED`
is enabled (including `--stereo`), belly mapping otherwise. Set it to `belly` or
`stereo` explicitly when publishing both streams. A missing stereo camera keeps
its left/right slot; it is never replaced by belly. To access belly and stereo
capture APIs simultaneously, use the separate instances described below.

Each camera/recording-name pair controls its own MP4. Multiple modules can
record the same camera at the same time using different names. Stopping one
name leaves the others recording; an unknown name never falls back to another
recording. The same name may be used independently on camera 1 and camera 2.
Reusing a name after stopping creates a new file.

### Named ROS captures (per module)

All seven services, including `health`, use
`camera_capture_interfaces/srv/Capture`:

```text
string name
---
bool success
string message
```

The endpoint supplies the camera, so there is no `camera` request field.
Pass `{name: default}` for health; health validates but otherwise ignores the
name. Empty/invalid names, unavailable cameras, duplicate starts and unknown
stops return `success: false` with the reason in `message`. Successful capture
responses include the saved path or recording state.

Rebuild the Docker image for the updated interface and services:

```bash
docker compose up -d --build
```

The image builds the interface, and its entrypoint automatically sources ROS
Jazzy and the installed interface. For an interactive CLI inside the container,
open a shell and source it explicitly (`docker exec` bypasses the entrypoint):

```bash
docker exec -it cameras-publisher bash
source /opt/camera_capture_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

Examples for two modules sharing camera 1:

```bash
ros2 service call /camera/capture/health camera_capture_interfaces/srv/Capture '{name: default}'
ros2 service call /camera/capture/image1 camera_capture_interfaces/srv/Capture '{name: inspection.snapshot}'
ros2 service call /camera/capture/image2 camera_capture_interfaces/srv/Capture '{name: inspection.snapshot}'

ros2 service call /camera/capture/video1_start camera_capture_interfaces/srv/Capture '{name: odometry}'
ros2 service call /camera/capture/video1_start camera_capture_interfaces/srv/Capture '{name: inspection}'
# Stop only odometry; inspection continues recording.
ros2 service call /camera/capture/video1_stop camera_capture_interfaces/srv/Capture '{name: odometry}'
ros2 service call /camera/capture/video1_stop camera_capture_interfaces/srv/Capture '{name: inspection}'

# Camera 2 is independent (available in stereo mode).
ros2 service call /camera/capture/video2_start camera_capture_interfaces/srv/Capture '{name: inspection}'
ros2 service call /camera/capture/video2_stop camera_capture_interfaces/srv/Capture '{name: inspection}'

# A default capture uses the same services with an explicit name.
ros2 service call /camera/capture/video1_start camera_capture_interfaces/srv/Capture '{name: default}'
ros2 service call /camera/capture/video1_stop camera_capture_interfaces/srv/Capture '{name: default}'
```

On a host/client machine, or when running the publisher without Docker, build
the interface from this repository root and source it before starting the node
or client. Rebuild both publisher and clients after updating the service definition.
Build tools are needed once:

```bash
sudo apt-get install build-essential python3-colcon-common-extensions ros-jazzy-rosidl-default-generators
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros_interfaces \
  --build-base build/ros_interfaces --install-base install/ros_interfaces --merge-install \
  --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
source install/ros_interfaces/setup.bash
export ROS_DOMAIN_ID=42
```

Alternatively, copy `ros_interfaces/camera_capture_interfaces` into the client
ROS workspace's `src/`, add `camera_capture_interfaces` as a dependency in that
client's `package.xml`, and build/source that workspace. A Python ROS module
uses the generated request with its existing node and executor:

```python
from camera_capture_interfaces.srv import Capture

start_client = node.create_client(Capture, '/camera/capture/video1_start')
start_client.wait_for_service()
future = start_client.call_async(Capture.Request(name='odometry'))
# Let the node's executor run; inspect future.result().success when complete.

# Later, stop the same name and inspect the response before treating it as saved.
stop_client = node.create_client(Capture, '/camera/capture/video1_stop')
stop_client.wait_for_service()
stop_future = stop_client.call_async(Capture.Request(name='odometry'))
```

For dual deployment, use `/camera/belly/capture` or `/camera/stereo/capture` as
the namespace. All ROS services remain available with `CP_HTTP_API_ENABLED=0`.

### HTTP API and OpenAPI

HTTP exposes only the seven commands in the table above. All are `POST`
requests with a required JSON body containing only `name`, including health:

```json
{"name": "odometry"}
```

Use `{"name":"default"}` for a default capture or a health query. Omitting the
body/name, an invalid name, or extra fields such as `camera` returns HTTP 422.
The endpoint selects the same camera slot as its ROS equivalent.

```bash
curl -X POST http://JETSON_IP:8000/api/health \
  -H 'Content-Type: application/json' -d '{"name":"default"}'
curl -X POST http://JETSON_IP:8000/api/image1 \
  -H 'Content-Type: application/json' -d '{"name":"inspection.snapshot"}'
curl -X POST http://JETSON_IP:8000/api/image2 \
  -H 'Content-Type: application/json' -d '{"name":"inspection.snapshot"}'
curl -X POST http://JETSON_IP:8000/api/video1_start \
  -H 'Content-Type: application/json' -d '{"name":"odometry"}'
curl -X POST http://JETSON_IP:8000/api/video1_start \
  -H 'Content-Type: application/json' -d '{"name":"inspection"}'
# Finalize odometry; inspection keeps recording.
curl -X POST http://JETSON_IP:8000/api/video1_stop \
  -H 'Content-Type: application/json' -d '{"name":"odometry"}'
curl -X POST http://JETSON_IP:8000/api/video1_stop \
  -H 'Content-Type: application/json' -d '{"name":"inspection"}'
curl -X POST http://JETSON_IP:8000/api/video2_start \
  -H 'Content-Type: application/json' -d '{"name":"inspection"}'
curl -X POST http://JETSON_IP:8000/api/video2_stop \
  -H 'Content-Type: application/json' -d '{"name":"inspection"}'
```

HTTP commands run on the ROS executor, serialized with ROS service callbacks
and recording timers. Both transports share the same recording state and
storage: a video started with HTTP `video1_start` can be stopped with ROS
`video1_stop` using the same name, and vice versa.

HTTP is enabled alongside ROS by default. Settings in `.env`:

```dotenv
CP_HTTP_API_ENABLED=1
CP_HTTP_API_HOST=0.0.0.0
CP_HTTP_API_PORT=8000
CP_HTTP_API_TIMEOUT=10
```

With Compose's host networking, HTTP listens directly on the Jetson's port;
no Docker port mapping is needed. Swagger UI is at `http://JETSON_IP:8000/docs`,
ReDoc at `/redoc`, and the OpenAPI JSON at `/openapi.json`.
The API has no authentication and is reachable on the configured bind address.
Set `CP_HTTP_API_HOST=127.0.0.1` for local access only, or
`CP_HTTP_API_ENABLED=0` to retain only ROS services.

Responses have `success` and `message`; health also includes a structured
`health` object and publishes the ROS health message. An unhealthy camera
report still returns HTTP 200 if the health command completed; inspect
`health.healthy`. Captures save to disk and return the path, not file contents.
Downloads/streaming are not exposed.

| HTTP status | Meaning |
|---|---|
| 200 | Command completed |
| 404 | Requested camera slot is unavailable |
| 409 | This camera/name pair is already recording, is not recording, or has no fresh frame |
| 422 | Invalid/missing capture name or unexpected JSON fields |
| 500 | Capture/encoder/storage failure |
| 503 | Command queue full or shutting down |
| 504 | Command execution exceeded `CP_HTTP_API_TIMEOUT` seconds |

Timed-out commands that have not started are cancelled. If execution has
already started, the response says the outcome is unknown; check health
before retrying. Shutdown rejects queued commands, stops HTTP, then finalizes
recordings. Port conflicts fail HTTP startup visibly in the module log.

### Health, encoding and storage

Subscribe with `ros2 topic echo /camera/capture/health` before calling `health`.
Each call publishes one `std_msgs/msg/String` message containing a UTC timestamp,
overall health and the numbered `cameras` mapping. Each camera entry includes
its label, availability/freshness, `recording`, active `recordings` (name-to-path
mapping), and `recording_errors` (name-to-error mapping). The same JSON is
returned in the ROS response and HTTP `health` object. `success` means the health
query completed; inspect `healthy` for camera health. The unused second belly
slot does not make health fail. Publication is on request.

Images use JPEG quality 90 (`CP_CAPTURE_JPEG_QUALITY`, 0–100). Videos use MPEG-4
Part 2 compression (`mp4v`) in `.mp4` files at 25 FPS
(`CP_CAPTURE_VIDEO_FPS`, positive). Both save at the published resolution.
Recording samples the latest frame at that rate, repeating it if capture is
slower. Frames older than 3 seconds are rejected. An encoding failure stops
only the affected recording; a camera stall or resolution change stops all
recordings using that camera and reports the reason per name in health/logs.
A recovered camera can be started again. Graceful shutdown finalizes all MP4s;
abrupt power loss or `SIGKILL` can leave an MP4 unfinished.

Files are stored under `CP_CAPTURE_DIRECTORY` (default `/workspace/captures`),
in `belly/`, `left/` or `right/` subdirectories, for example:

```text
/workspace/captures/left/image_20261015T140000_123456Z_a1b2c3d4.jpeg
/workspace/captures/right/video_20261015T140001_123456Z_e5f6a7b8.mp4
/workspace/captures/belly/video_odometry_20261015T140002_123456Z_f6a7b8c9.mp4
```

Filenames contain the supplied capture name (omitted for `default`), UTC
date/time with microseconds and a random suffix to avoid overwriting files.
Capture names are case-sensitive, 1–64 characters, start with an ASCII letter
or digit, and contain only ASCII letters, digits, dots, dashes or underscores.
They are IDs/prefixes; the server supplies the extension and storage directory.

The Dockerfile declares `/workspace/captures` as a volume. Compose mounts the
named `camera-captures` volume for both deployment modes. Files survive restarts,
rebuilds and `docker compose down`; **`docker compose down -v` deletes the volume
and captures**. Copy files to the host with:

```bash
docker cp cameras-publisher:/workspace/captures ./saved-captures
```

Compose uses `CP_CAPTURE_DIRECTORY` from `.env` as the capture volume's target.
For a host folder, replace the mount with
`./captures:${CP_CAPTURE_DIRECTORY:-/workspace/captures}`. Non-Docker runs need
a writable directory, e.g. `CP_CAPTURE_DIRECTORY=./captures`, ROS `std_msgs`, and
the built `camera_capture_interfaces` overlay (build/source instructions above).

Run state/adapter tests without cameras, ROS or OpenCV:
`python3 -m unittest -v test_camera_api`. With development requirements installed,
run `python3 -m unittest -v test_camera_api test_camera_http` for HTTP/OpenAPI,
concurrency, dual-server and real JPEG/MP4 tests. To exercise real ROS service
serialization/discovery/callbacks using synthetic frames, source ROS and the
interface overlay, then run `python3 -m unittest -v test_camera_ros`.
Jetson hardware and DDS across deployment machines still need validation.

### Separate belly and stereo modules at the same time

Use the standalone dual-instance Compose file. When switching from the single
module deployment, stop that deployment first to release its camera devices
and HTTP port. Do not use `-v`, which would delete saved captures.

```bash
docker compose down
docker compose -f docker-compose.dual.yml up -d --build
docker compose -f docker-compose.dual.yml logs -f
```

| Module | Cameras | HTTP / Swagger | ROS capture namespace | ROS node |
|---|---|---|---|---|
| Belly | Belly only | `http://JETSON_IP:8000/docs` | `/camera/belly/capture` | `cameras_publisher_belly` |
| Stereo | Left and right only | `http://JETSON_IP:8001/docs` | `/camera/stereo/capture` | `cameras_publisher_stereo` |

Both HTTP APIs and both ROS APIs run simultaneously. Each module owns its
cameras and recording state. For example, `POST :8000/api/video1_start`
records belly; `POST :8001/api/video1_start` records left independently.
ROS equivalents are `/camera/belly/capture/video1_start` and
`/camera/stereo/capture/video1_start`.
All use `camera_capture_interfaces/srv/Capture` with only `name` in the request.

The dual file fixes camera selection and API mode per service. Both use the
same `ROS_DOMAIN_ID` and named capture volume, with files separated into
`belly/`, `left/` and `right/`; logs are in `logs/belly/` and `logs/stereo/`.
Each container restarts independently. Configure distinct ports and ROS
namespaces in `.env`:

```dotenv
CP_BELLY_HTTP_API_PORT=8000
CP_STEREO_HTTP_API_PORT=8001
CP_BELLY_CAPTURE_API_NAMESPACE=/camera/belly/capture
CP_STEREO_CAPTURE_API_NAMESPACE=/camera/stereo/capture
```

Apply changes with `docker compose -f docker-compose.dual.yml up -d`.
For independently launched instances outside this dual file, assign each
instance its own `CP_HTTP_API_PORT`, `CP_CAPTURE_API_NAMESPACE` and
`CP_NODE_NAME`, as well as the correct camera selection/mode.

## Settings
- `ROS_DOMAIN_ID` (default **42** in `.env`) must be the same for every SWARMER module and for PX4's uXRCE-DDS agent; set PX4's `UXRCE_DDS_DOM_ID` to match. The RMW is Cyclone DDS.
- Almost every setting of `config.py` can be changed in `.env` as `CP_<NAME>`: the prefix `CP_` (= this module's `config.py`; visual odometry uses `VO_`, obstacle detection `OD_`) added to the setting's name, e.g. `CP_BELLY_CAMERA_EXPOSURE=30`. Then run `docker compose up -d` (no rebuild). The exceptions are fixed values such as `BELLY_CAMERA_DISTORTION_COEFFS`; the ROS domain is the plain variable `ROS_DOMAIN_ID`.
- **Belly lens calibration** (once per camera, checkerboard at 640x400): `fx`, `fy`, `cx`, `cy` as `CP_BELLY_CAMERA_FX/FY/CX/CY` in `.env` (then `docker compose up -d`); the 5 distortion coefficients as `BELLY_CAMERA_DISTORTION_COEFFS` in `config.py` (then `docker compose up -d --build`), they cannot be set with a `CP_` variable. Uncalibrated, the visual odometry has a scale error of about 8 %.
- Measure and set the mounts: `CP_BELLY_CAMERA_MOUNT_*_DEG`, `CP_BELLY_CAMERA_T*_M` (offset from the IMU) and `CP_STEREO_CAMERA_BASELINE_CMS` (6 cm).
- Fixed belly exposure (less motion blur): `CP_BELLY_CAMERA_EXPOSURE: "30"` (100 µs units; default auto).

## Logs
Written in the container to **`/workspace/logs/publish_cameras.log`** (DEBUG: devices found, every open/re-open, intrinsics, stalls, errors, fps per stream every 5 s; rotates at 5 MB, keeps `.1`-`.3`). `docker-compose.yml` mounts the folder as `logs/` next to it on the Jetson:
```bash
tail -f /opt/swarmer/cameras_publisher/logs/publish_cameras.log
docker exec cameras-publisher tail -f /workspace/logs/publish_cameras.log      # the same, from inside
```
`docker compose logs -f` shows the INFO part of the same log.

## Check
```bash
ros2 topic hz /camera/belly/image_compressed     # ~30 Hz
ros2 topic hz /camera/left/image_compressed      # ~25 Hz
```
Raw check of the CSI cameras, no ROS: `xhost +local:docker && tools/test_stereo_camera.sh`.
