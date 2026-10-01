"""Pure quaternion math (wxyz, Hamilton product) -- no mujoco import (testbed.control never
imports mujoco). Cross-checked numerically against mujoco.mju_mulQuat/mju_rotVecQuat/mju_negQuat
over 200 random unit quaternions [measured: scratch probe, 2026-09-18] -- max mul error 0.0,
max rotate error 1.1e-15 -- so testbed.sim can use this in place of the mujoco helpers wherever
control-package code (or code that must stay mujoco-free) needs the same quaternion algebra.
to_rotvec/from_rotvec (axis*angle, rad) are the orientation-error representation the IK
uses; yaw_rad extracts heading alone for HUD/twist reporting."""
import numpy as np


def conj(q: np.ndarray) -> np.ndarray:
    """Conjugate of a wxyz unit quaternion (its inverse)."""
    q = np.asarray(q, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]])


def mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product a * b, both wxyz."""
    w1, x1, y1, z1 = np.asarray(a, dtype=float)
    w2, x2, y2, z2 = np.asarray(b, dtype=float)
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate the 3-vector v by the wxyz unit quaternion q: q * (0, v) * conj(q)."""
    v = np.asarray(v, dtype=float)
    qv = np.array([0.0, v[0], v[1], v[2]])
    return mul(mul(q, qv), conj(q))[1:]


def to_rotvec(q: np.ndarray) -> np.ndarray:
    """Wxyz unit quaternion -> rotation vector (axis * angle, rad), the shortest-angle
    representative (angle in [0, pi], w >= 0 sign chosen before extracting it)."""
    q = np.asarray(q, dtype=float)
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    n = np.linalg.norm(q[1:])
    angle = 2.0 * np.arctan2(n, q[0])
    return np.zeros(3) if n < 1e-12 else q[1:] / n * angle


def from_rotvec(r: np.ndarray) -> np.ndarray:
    """Rotation vector (axis * angle, rad) -> wxyz unit quaternion."""
    r = np.asarray(r, dtype=float)
    a = np.linalg.norm(r)
    if a < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return np.array([np.cos(a / 2.0), *(np.sin(a / 2.0) * r / a)])


def from_matrix(R: np.ndarray) -> np.ndarray:
    """Rotation matrix (3x3, columns = axes) -> wxyz unit quaternion (Shepperd's method,
    the standard branch-on-largest-diagonal-term construction used for placement_skills'
    grasp-frame axes, since those are built column-by-column rather than as an
    axis-angle)."""
    R = np.asarray(R, dtype=float)
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1.0) * 2
        return np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0) * 2
    q = np.zeros(4)
    q[0] = (R[k, j] - R[j, k]) / s
    q[1 + i] = 0.25 * s
    q[1 + j] = (R[j, i] + R[i, j]) / s
    q[1 + k] = (R[k, i] + R[i, k]) / s
    return q


def yaw_rad(q: np.ndarray) -> float:
    """Yaw (rotation about z, rad) extracted from a wxyz unit quaternion, ignoring
    roll/pitch -- the base heading, e.g. for HUD/twist reporting in B/W frames."""
    w, x, y, z = np.asarray(q, dtype=float)
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
