"""Preserve point representatives and geometry while accelerating window voxelization."""
import numpy as np
from mid360_bringup.local_map_window import voxel_indices


def test_first_representative_with_negative_coordinates_and_duplicates():
    rng = np.random.default_rng(20261124)
    points = rng.normal(size=(5000, 3)).astype(np.float32)
    points = np.vstack((points, points[:2000]))
    for voxel in (.05, .15, 1.0):
        cells = np.floor(points / voxel).astype(np.int64)
        first = {}
        for index, cell in enumerate(cells):
            first.setdefault(tuple(cell), index)
        np.testing.assert_array_equal(voxel_indices(points, voxel), sorted(first.values()))


def test_wide_coordinates_do_not_alias_voxels_when_packing_would_overflow():
    points = np.array([[0., 0., 0.], [1e10, -1e10, 1e10],
                       [-1e10, 1e10, -1e10], [0., 0., 0.]])
    np.testing.assert_array_equal(voxel_indices(points, .05), [0, 1, 2])
    assert len(voxel_indices(np.empty((0, 3)), .05)) == 0
