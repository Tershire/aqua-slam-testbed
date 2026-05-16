#!/usr/bin/env python3
"""Acoustic noise post-processor: Stonefish depthcamera → Water Linked SONAR 3D-15.

Uses a Stonefish ``depthcamera`` sensor (90°H × 40°V, 0.2–15 m) to avoid the
coordinate-frame ambiguity of ``multibeam2d`` tile assembly.  The depth image is
backprojected to a 3-D point cloud in the sensor frame, then three OceanSim-inspired
acoustic effects are added:

  1. Range-dependent Rayleigh dropout  — simulates 1.2 MHz absorption + spreading loss.
  2. Angular jitter (Gaussian)          — beam uncertainty: 0.85° H / 1.60° V (1-σ).
  3. Rayleigh intensity field           — acoustic speckle, range-attenuated.

Frame transform
---------------
The depthcamera shares rpy="π/2 0 π/2" with camera_left → identical sensor
orientation.  ``bluerov/base_link`` (= camera_left tracking frame in AQUA-SLAM) is
reached by a pure translation from the sonar origin:

    NED delta camera_left → sonar: (Δnorth=0.05, Δeast=0.06, Δdown=0)
    In shared sensor frame (x=east, y=down, z=north): T = [0.06, 0, 0.05]

Points are then forwarded to the aqua_slam world frame via TF.

Topics
------
  Sub : /girona500/sonar_3d15/raw/image_depth   sensor_msgs/Image  (32FC1, z-depth [m])
  Pub : /sonar_3d15/points                       sensor_msgs/PointCloud2  (xyzi, aqua_slam)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import tf2_ros
from sensor_msgs.msg import Image, PointCloud2, PointField

# ── Camera intrinsics (simple_tank.scn: hfov=90°, 256×W, square pixels) ──────
# Actual resolution is determined at runtime from the first message; grids are
# built lazily in _build_grids().  Constants below match the scenario spec.
_FX = 128.0          # = (width/2) / tan(hfov/2) = 256/2 / tan(45°)
_FY = 128.0          # square pixels
_RANGE_MIN = 0.2     # [m]  matches depth_min in scenario
_RANGE_MAX = 15.0    # [m]  matches depth_max in scenario

# Lazily-initialised per-pixel grids (built on first message)
_DX: np.ndarray | None = None
_DY: np.ndarray | None = None
_IMG_W: int = 0
_IMG_H: int = 0


def _build_grids(w: int, h: int) -> None:
    global _DX, _DY, _IMG_W, _IMG_H
    if _IMG_W == w and _IMG_H == h:
        return
    cx = (w - 1) / 2.0
    cy = (h - 1) / 2.0
    U, V = np.meshgrid(np.arange(w, dtype=np.float32),
                       np.arange(h, dtype=np.float32))
    _DX = (U - cx) / _FX
    _DY = (V - cy) / _FY
    _IMG_W, _IMG_H = w, h

# ── Sensor → bluerov/base_link transform ─────────────────────────────────────
#
# Sensor frame (depthcamera, same rpy as stereo cameras): RDF in NED body.
#   x = NED-east (right), y = NED-down, z = NED-north (forward)
#   NOTE: Stonefish depth images use GL bottom-up storage (no y-flip), so the
#         backprojected y_s is negated below to restore RDF y = NED-down.
#
# bluerov/base_link = body FLU (x=forward, y=left, z=up).
# R_body_sensor transforms sensor RDF → body FLU:
#   sensor_z (NED-north)  → body_x  (forward)
#   sensor_x (NED-east)   → -body_y (right = -left)
#   sensor_y (NED-down)   → -body_z (down = -up)
#
# Sonar at body NED (0.55, 0, 0) → body FLU: (0.55, 0, 0)
_R_BODY_SENSOR = np.array([
    [ 0.0,  0.0,  1.0],
    [-1.0,  0.0,  0.0],
    [ 0.0, -1.0,  0.0],
], dtype=np.float64)
_T_BODY_SENSOR = np.array([0.55, 0.0, 0.0], dtype=np.float64)

# ── Acoustic parameters (3D-15, low-frequency mode) ──────────────────────────
_ATTEN_COEFF    = 0.02                    # survival prob = exp(-k·r); ~26% dropout @ 15m
_SIGMA_H_RAD    = np.radians(0.85 / 2)   # H beam jitter 1-σ [rad / unit range]
_SIGMA_V_RAD    = np.radians(1.60 / 2)   # V beam jitter 1-σ [rad / unit range]
_RAYLEIGH_BASE  = 0.8
_RAYLEIGH_SLOPE = 0.04                    # σ(r) = max(0.05, BASE − SLOPE·r)

# ── PointCloud2 output format ─────────────────────────────────────────────────
_FIELDS_XYZI = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16


class SimSonar3D15Converter(Node):
    def __init__(self):
        super().__init__('sim_sonar_3d15_converter')
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._sub = self.create_subscription(
            Image, '/girona500/sonar_3d15/raw/image_depth', self._cb, 10)
        self._pub = self.create_publisher(PointCloud2, '/sonar_3d15/points', 10)

    def _cb(self, msg: Image):
        # Decode 32FC1 depth image (z-depth in metres, NOT spherical range)
        _build_grids(msg.width, msg.height)
        depth = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.height, msg.width)

        # Backproject to sensor RDF frame (x=right/NED-east, y=down/NED-down, z=fwd/NED-north).
        # Stonefish publishes depth images with standard orientation (v=0 at top = NED-up),
        # so the standard pinhole formula gives y > 0 = NED-down directly.
        x_s = _DX * depth                  # (H, W)
        y_s =  _DY * depth                 # no negation needed
        z_s = depth                        # z-depth = forward distance

        # Compute spherical range; keep only points within sonar spec
        rng = np.sqrt(x_s**2 + y_s**2 + z_s**2)
        valid = (rng >= _RANGE_MIN) & (rng <= _RANGE_MAX) & (depth > 0)
        pts = np.column_stack([
            x_s[valid].ravel(),
            y_s[valid].ravel(),
            z_s[valid].ravel(),
        ]).astype(np.float64)
        rng = rng[valid].ravel()

        if len(pts) == 0:
            return

        pts_xyzi = _apply_acoustic_noise(pts, rng)
        if len(pts_xyzi) == 0:
            return

        # Look up aqua_slam → bluerov/base_link (camera tracking frame)
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                'aqua_slam', 'bluerov/base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except Exception:
            return   # SLAM not yet initialised; drop sweep

        tr = tf_stamped.transform.translation
        q  = tf_stamped.transform.rotation
        R_w_body = _quat_to_rot(q.x, q.y, q.z, q.w)
        t_w_body = np.array([tr.x, tr.y, tr.z])

        # sensor RDF → body FLU → aqua_slam
        xyz_s = pts_xyzi[:, :3].T.astype(np.float64)              # (3, N)
        xyz_b = _R_BODY_SENSOR @ xyz_s + _T_BODY_SENSOR[:, None]  # (3, N)
        xyz_w = R_w_body @ xyz_b + t_w_body[:, None]              # (3, N)

        out = np.vstack([xyz_w, pts_xyzi[:, 3]]).T.astype(np.float32)  # (N, 4)
        self._pub.publish(_pack_xyzi(msg.header.stamp, 'aqua_slam', out))


# ── Acoustic noise model ──────────────────────────────────────────────────────

def _apply_acoustic_noise(pts: np.ndarray, rng: np.ndarray) -> np.ndarray:
    """Return (M, 4) float32 [x, y, z, intensity] after acoustic effects.

    Sensor frame: x=right (H sweep), y=down (V sweep), z=forward (boresight).
    Angular jitter is applied in x (H) and y (V) proportional to range.
    """
    keep = np.random.random(len(pts)) < np.exp(-_ATTEN_COEFF * rng)
    pts, rng = pts[keep], rng[keep]
    if len(pts) == 0:
        return np.empty((0, 4), dtype=np.float32)

    pts[:, 0] += np.random.normal(0.0, _SIGMA_H_RAD, len(pts)) * rng
    pts[:, 1] += np.random.normal(0.0, _SIGMA_V_RAD, len(pts)) * rng

    sigma     = np.maximum(0.05, _RAYLEIGH_BASE - _RAYLEIGH_SLOPE * rng)
    intensity = np.clip(np.random.rayleigh(sigma), 0.0, 1.0).astype(np.float32)

    return np.column_stack([pts, intensity]).astype(np.float32)


# ── PointCloud2 helpers ───────────────────────────────────────────────────────

def _pack_xyzi(stamp, frame_id: str, pts: np.ndarray) -> PointCloud2:
    msg = PointCloud2()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.height      = 1
    msg.width       = len(pts)
    msg.fields      = _FIELDS_XYZI
    msg.is_bigendian = False
    msg.point_step  = _POINT_STEP
    msg.row_step    = _POINT_STEP * len(pts)
    msg.is_dense    = True
    msg.data        = pts.astype(np.float32).tobytes()
    return msg


def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - w*z),     2*(x*z + w*y)],
        [    2*(x*y + w*z), 1 - 2*(x*x + z*z),     2*(y*z - w*x)],
        [    2*(x*z - w*y),     2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SimSonar3D15Converter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
