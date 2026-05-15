#!/usr/bin/env python3
"""Acoustic noise post-processor: Stonefish multibeam2d → Water Linked SONAR 3D-15.

Stonefish's multibeam2d provides geometrically accurate range measurements but
has no acoustic physics.  This node adds three OceanSim-inspired effects:

  1. Range-dependent Rayleigh dropout  — simulates 1.2 MHz absorption + spreading loss.
  2. Angular jitter (Gaussian)          — beam uncertainty: 0.85° H / 1.60° V (1-σ half-width).
  3. Rayleigh intensity field           — acoustic speckle, range-attenuated.

Topics
------
  Sub : /girona500/sonar_3d15/raw   sensor_msgs/PointCloud2  (xyz, sensor frame)
  Pub : /sonar_3d15/points          sensor_msgs/PointCloud2  (xyzi, sensor frame)
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

# ── Acoustic parameters (3D-15, low-frequency mode) ──────────────────────────

# Attenuation: survival prob P(r) = exp(-COEFF * r).
# 1.2 MHz in seawater ≈ 0.05 dB/m absorption + spherical spreading;
# COEFF=0.02 gives ~26 % dropout at 15 m — mild, consistent with SLAM use.
_ATTEN_COEFF = 0.02

# Beam angular uncertainty (1-σ): half of datasheet angular resolution.
#   H: 0.85° / 2 → 0.00742 rad
#   V: 1.60° / 2 → 0.01396 rad
_SIGMA_H_RAD = np.radians(0.85 / 2)
_SIGMA_V_RAD = np.radians(1.60 / 2)

# Rayleigh intensity speckle: σ(r) = BASE − SLOPE·r (decays with range).
# At r=0: σ=0.8 → mean intensity ≈ 1.0.  At r=15 m: σ=0.2.
_RAYLEIGH_BASE  = 0.8
_RAYLEIGH_SLOPE = 0.04   # (0.8 - 0.04·15 = 0.2)

# ─────────────────────────────────────────────────────────────────────────────

_FIELDS_XYZI = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]
_POINT_STEP = 16  # 4 × float32


class SimSonar3D15Converter(Node):
    def __init__(self):
        super().__init__('sim_sonar_3d15_converter')
        self._sub = self.create_subscription(
            PointCloud2, '/girona500/sonar_3d15/raw', self._cb, 10)
        self._pub = self.create_publisher(PointCloud2, '/sonar_3d15/points', 10)

    def _cb(self, msg: PointCloud2):
        pts = _unpack_xyz(msg)          # (N, 3) float32
        if len(pts) == 0:
            return
        pts_xyzi = _apply_acoustic_noise(pts)
        if len(pts_xyzi) == 0:
            return
        self._pub.publish(_pack_xyzi(msg.header, pts_xyzi))


# ── PointCloud2 helpers ───────────────────────────────────────────────────────

def _unpack_xyz(msg: PointCloud2) -> np.ndarray:
    """Return (N, 3) float32 array from an xyz PointCloud2."""
    n = msg.width * msg.height
    if n == 0:
        return np.empty((0, 3), dtype=np.float32)
    step = msg.point_step
    off = {f.name: f.offset for f in msg.fields}
    # Reshape to (N, step) byte array; flatten each 4-byte field column → float32.
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
    pts = np.empty((n, 3), dtype=np.float32)
    for col, name in enumerate(('x', 'y', 'z')):
        o = off[name]
        pts[:, col] = raw[:, o:o + 4].flatten().view(np.float32)
    return pts


def _pack_xyzi(header, pts: np.ndarray) -> PointCloud2:
    """Pack (N, 4) float32 [x, y, z, intensity] into a PointCloud2."""
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = _FIELDS_XYZI
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * len(pts)
    msg.data = pts.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


# ── Acoustic noise model ──────────────────────────────────────────────────────

def _apply_acoustic_noise(pts: np.ndarray) -> np.ndarray:
    """Return (M, 4) float32 [x, y, z, intensity] after acoustic effects.

    Sensor-frame convention (Stonefish multibeam2d after rpy=1.5708 0 1.5708):
      z — forward (boresight), x — horizontal (right), y — vertical (down).
    Angular jitter is applied in x (H) and y (V) proportional to range.
    """
    rng = np.linalg.norm(pts, axis=1)          # (N,) per-point range [m]

    # 1. Range-dependent Rayleigh dropout.
    keep = np.random.random(len(pts)) < np.exp(-_ATTEN_COEFF * rng)
    pts, rng = pts[keep], rng[keep]
    if len(pts) == 0:
        return np.empty((0, 4), dtype=np.float32)

    # 2. Gaussian angular jitter (beam uncertainty, first-order Cartesian approx).
    #    Δx ≈ r · δθ_H,  Δy ≈ r · δθ_V,  Δz ≈ 0  (valid for small angles).
    pts[:, 0] += np.random.normal(0.0, _SIGMA_H_RAD, len(pts)) * rng
    pts[:, 1] += np.random.normal(0.0, _SIGMA_V_RAD, len(pts)) * rng

    # 3. Rayleigh intensity: σ decreases with range (weaker far returns).
    sigma = np.maximum(0.05, _RAYLEIGH_BASE - _RAYLEIGH_SLOPE * rng)
    intensity = np.random.rayleigh(sigma).astype(np.float32)
    # Normalise so that near-range mean ≈ 1.0  (σ=0.8 → mean = 0.8·√(π/2) ≈ 1.0).
    intensity = np.clip(intensity, 0.0, 1.0)

    return np.column_stack([pts, intensity]).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SimSonar3D15Converter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
