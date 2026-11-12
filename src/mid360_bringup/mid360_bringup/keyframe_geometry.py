"""Pose and hashing helpers shared by the online keyframe candidate publisher."""

from bisect import bisect_left
import hashlib

import numpy as np
from scipy.spatial.transform import Rotation


def ns(stamp):
    return stamp.sec * 10**9 + stamp.nanosec


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b''):
            digest.update(block)
    return digest.hexdigest()


def message_pose(pose):
    quaternion = np.array([pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1) > .01:
        raise ValueError('Invalid pose quaternion')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(transform).all():
        raise ValueError('Invalid pose position')
    return transform


def interpolate(stamps, poses, stamp, max_gap_ns):
    index = bisect_left(stamps, stamp)
    if index < len(stamps) and stamps[index] == stamp:
        return poses[index]
    if index == 0 or index == len(stamps) or stamps[index] - stamps[index - 1] > max_gap_ns:
        return None
    a, b = poses[index - 1], poses[index]
    fraction = (stamp - stamps[index - 1]) / (stamps[index] - stamps[index - 1])
    rotation = Rotation.from_matrix(a[:3, :3].T @ b[:3, :3])
    out = np.eye(4)
    out[:3, :3] = a[:3, :3] @ Rotation.from_rotvec(rotation.as_rotvec() * fraction).as_matrix()
    out[:3, 3] = (1 - fraction) * a[:3, 3] + fraction * b[:3, 3]
    return out


def transformed(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]
