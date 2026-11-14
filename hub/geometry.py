"""Poses: a rotation and a translation, as 4x4 matrices, with the few operations the hub needs.

Naming: T_a_b takes a point in frame b to frame a (p_a = T_a_b @ p_b), so T_a_c = T_a_b @ T_b_c.
"""
from __future__ import annotations

import math

import numpy as np


def pose(R=None, t=None) -> np.ndarray:
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = np.asarray(R, float)
    if t is not None:
        T[:3, 3] = np.asarray(t, float).ravel()
    return T


def inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    return pose(R.T, -R.T @ t)


def rot_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rodrigues(rvec) -> np.ndarray:
    """Rotation vector (axis x angle) -> matrix."""
    r = np.asarray(rvec, float).ravel()
    a = float(np.linalg.norm(r))
    if a < 1e-12:
        return np.eye(3)
    k = r / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * (K @ K)


def rotvec(R) -> np.ndarray:
    """Matrix -> rotation vector."""
    R = np.asarray(R, float)
    c = np.clip((np.trace(R) - 1) / 2, -1.0, 1.0)
    a = math.acos(c)
    if a < 1e-9:
        return np.zeros(3)
    if math.pi - a < 1e-6:                              # near half a turn: take the axis from R + I
        v = np.sqrt(np.maximum((np.diag(R) + 1) / 2, 0.0))
        k = int(np.argmax(v))
        axis = (R[:, k] + np.eye(3)[:, k]) / (2 * v[k])
        return axis / np.linalg.norm(axis) * a
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * math.sin(a))
    return axis * a


def angle_between(Ra, Rb) -> float:
    """Angle of the rotation that takes one orientation to the other [rad]."""
    return float(np.linalg.norm(rotvec(np.asarray(Ra, float).T @ np.asarray(Rb, float))))


def from_axes(z, y) -> np.ndarray:
    """Rotation whose third column is z and second is (as near as possible) y."""
    z = np.asarray(z, float) / np.linalg.norm(z)
    y = np.asarray(y, float)
    y = y - z * float(y @ z)
    y /= np.linalg.norm(y)
    return np.column_stack([np.cross(y, z), y, z])


def yaw_of(T: np.ndarray) -> float:
    return math.atan2(T[1, 0], T[0, 0])
