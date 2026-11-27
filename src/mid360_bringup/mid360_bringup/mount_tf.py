"""Compose the robot mount without changing FAST-LIO's IMU state or clouds."""
import numpy as np
from scipy.spatial.transform import Rotation


def go2_transforms(imu_translation, imu_rotation, mount):
    """Return T_imu_base and T_base_lidar, with T_imu_lidar unchanged."""
    xyz = np.asarray(mount['xyz'], dtype=float)
    rpy = np.asarray(mount['rpy'], dtype=float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not (
            np.isfinite(xyz).all() and np.isfinite(rpy).all()):
        raise ValueError('Go2 mount xyz and rpy must each contain three finite values')
    base_lidar = np.eye(4)
    base_lidar[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    base_lidar[:3, 3] = xyz
    imu_lidar = np.eye(4)
    imu_lidar[:3, :3] = imu_rotation
    imu_lidar[:3, 3] = imu_translation
    return imu_lidar @ np.linalg.inv(base_lidar), base_lidar


def static_arguments(parent, child, transform):
    values = [*transform[:3, 3], *Rotation.from_matrix(transform[:3, :3]).as_quat()]
    args = ['--frame-id', parent, '--child-frame-id', child]
    for flag, value in zip(['--x', '--y', '--z', '--qx', '--qy', '--qz', '--qw'], values):
        args.extend([flag, str(value)])
    return args
