#!/usr/bin/env python3
"""Write the stereo camera + IMU calibration YAML requested by challenge
organisers (e.g. ORDEAL, re: the Chania .mcap submission): intrinsics (focal
length, principal point, distortion model + coefficients) for both cameras,
the left/right extrinsics, the IMU->camera rigid-body transform (T_cam_imu),
and the IMU-camera time offset if known.

Every number comes from config.py - the same constants
the obstacle detection's stereo pipeline and publish_cameras.py's
camera_info/tf_static topics use - so this file and the topics baked into the
.mcap can never disagree. See config.py's "Calibration" section for where each
value comes from and which ones (the IMU->camera transform, the time offset)
are still an unmeasured placeholder rather than a real measurement.

Usage:
    python3 write_calibration_yaml.py [-o calibration.yaml]
"""

import argparse
import math
from datetime import datetime, timezone

import yaml

import config


def _intrinsics():
    """(fx, fy, cx, cy) exactly as node_obstacle_detection.py's
    stereo_depth_computation_worker derives them: a single focal length from
    the horizontal FOV and processing width, assumed square pixels."""
    focal = config.IMG_W_PROCESS / (2.0 * math.tan(math.radians(config.FOV_D / 2.0)))
    cx = config.IMG_W_PROCESS / 2.0
    cy = config.IMG_H_PROCESS / 2.0
    return focal, focal, cx, cy


def _camera_block(frame_id):
    fx, fy, cx, cy = _intrinsics()
    k1, k2, p1, p2, k3 = config.CALIBRATION_DISTORTION_COEFFS
    return {
        'frame_id': frame_id,
        'image_width': config.IMG_W_PROCESS,
        'image_height': config.IMG_H_PROCESS,
        'focal_length_px': {'fx': fx, 'fy': fy},
        'principal_point_px': {'cx': cx, 'cy': cy},
        'distortion_model': config.CALIBRATION_DISTORTION_MODEL,
        'distortion_coefficients': {'k1': k1, 'k2': k2, 'p1': p1, 'p2': p2, 'k3': k3},
        'distortion_note': (
            "Zero because the running pipeline treats the feed as an "
            "already-undistorted pinhole model (see "
            "node_obstacle_detection.py stereo_depth_computation_worker, "
            "which calls cv2.stereoRectify/initUndistortRectifyMap with "
            "zero distCoeffs) - this is what the software uses, not an "
            "independently measured lens distortion."
        ),
    }


def build_calibration():
    baseline_m = config.STEREO_CAMERA_BASELINE_CMS / 100.0
    return {
        'rig': 'Waveshare/Seeed IMX219-83 stereo camera',
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'cameras': {
            'left': _camera_block('camera_left'),
            'right': dict(
                _camera_block('camera_right'),
                extrinsics_relative_to_left={
                    'translation_m': {'x': baseline_m, 'y': 0.0, 'z': 0.0},
                    'rotation_xyzw': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                    'note': (
                        "Parallel/rectified stereo pair assumption: baseline "
                        "is a pure horizontal (camera-x) translation, no "
                        "relative rotation. Baseline is "
                        "config.STEREO_CAMERA_BASELINE_CMS "
                        f"({config.STEREO_CAMERA_BASELINE_CMS} cm), the "
                        "REAL MEASURED rig value."
                    ),
                },
            ),
        },
        'imu_to_camera': {
            'description': 'T_cam_imu: rigid-body transform from the IMU to the left camera optical frame',
            'reference_camera': 'left',
            'measured': config.CALIBRATION_IMU_TO_CAMERA_MEASURED,
            'translation_m': {
                'x': config.CALIBRATION_IMU_TO_CAMERA_TX_M,
                'y': config.CALIBRATION_IMU_TO_CAMERA_TY_M,
                'z': config.CALIBRATION_IMU_TO_CAMERA_TZ_M,
            },
            'rotation_xyzw': {
                'x': config.CALIBRATION_IMU_TO_CAMERA_QX,
                'y': config.CALIBRATION_IMU_TO_CAMERA_QY,
                'z': config.CALIBRATION_IMU_TO_CAMERA_QZ,
                'w': config.CALIBRATION_IMU_TO_CAMERA_QW,
            },
            'note': (
                "Real measurement - see config.py's CALIBRATION_IMU_TO_CAMERA_*."
                if config.CALIBRATION_IMU_TO_CAMERA_MEASURED else
                "PLACEHOLDER (identity/zero) - the mount offset between the IMU "
                "and the camera has not been physically measured yet. Set "
                "config.py's CALIBRATION_IMU_TO_CAMERA_* (or the matching "
                "CP_CALIBRATION_IMU_TO_CAMERA_* env vars) from calipers/CAD and "
                "flip CALIBRATION_IMU_TO_CAMERA_MEASURED to true before this "
                "file is treated as a real calibration."
            ),
        },
        'imu_camera_time_offset_sec': config.CALIBRATION_IMU_CAMERA_TIME_OFFSET_SEC,
        'imu_camera_time_offset_note': (
            "camera_stamp - imu_stamp. null = not measured (no hardware "
            "trigger log / sync test has been run yet), reported as unknown "
            "rather than assumed 0."
        ),
    }


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('-o', '--output', default='calibration.yaml',
                    help="Output YAML path (default: calibration.yaml, next to this script; "
                         "point it at the folder holding the .mcap).")
    return p.parse_args()


def main():
    args = _parse_args()
    calibration = build_calibration()
    with open(args.output, 'w') as f:
        yaml.safe_dump(calibration, f, sort_keys=False, default_flow_style=False)
    print(f"[write_calibration_yaml] Wrote {args.output}")
    if not config.CALIBRATION_IMU_TO_CAMERA_MEASURED:
        print("[write_calibration_yaml] WARNING: imu_to_camera is still the "
              "identity/zero PLACEHOLDER, not a real measurement - see "
              "config.py's Calibration section before sending this file out.")
    if config.CALIBRATION_IMU_CAMERA_TIME_OFFSET_SEC is None:
        print("[write_calibration_yaml] NOTE: imu_camera_time_offset_sec is "
              "unknown (null) - fill in config.CALIBRATION_IMU_CAMERA_TIME_OFFSET_SEC "
              "if/when it gets measured.")


if __name__ == '__main__':
    main()
