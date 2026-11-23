# MID-360 experimental localization tools

This source bundle preserves the independently versioned `mid360_lidar` workflow (source commit `e2d3bd1adf799f7f1372653db71c540f29601e1a`). The standard `src/mid360_bringup` package and the main navigation launch remain unchanged. Use only one sensor launch per experiment.

## Experimental entry

Build Sensor and the matching Localization branch first. For sibling workspaces, set `NAV3_LOCALIZATION_WS` if its directory name differs. For a Nav3 checkout with modules, the shared `data/maps/building_split_space.pcd` is preferred. `NAV3_MAP_PCD` overrides the map explicitly.

```bash
bash tools/mid360_lidar/scripts/start_scan.sh --mode race --region space_2
```

Race runs private BBS and measured-keyframe validators with independent ambiguity/two-window checks. Only the authority applies the winning map-to-odom transform and publishes navigation permission. Prior tracking follows; a tracking failure starts a new race. KISS fallback remains disabled. BBS asynchronous receipt and query-wait protection are retained.

After recording closes, strict automatic library admission can create a version for the next run; the frozen 40-frame baseline is unchanged and at most 24 automatic frames are retained. Use `--no-auto-library-update` for independent comparisons. Automatic poses are pseudo-labels, not manual floor verification.

## Required data and build products

Git contains source, configuration, manifests and indices, not map PCds, reference-frame PCds/NPZ descriptors, BBS `target.bin`, recordings or build products. Transfer matching data to the same relative paths under `reference_maps/` before running. Build BBS with `/usr/bin/python3 scripts/prepare_bbs_model.py --build-only`. Exact production map caches are prepared by the matching Localization `prepare_region_map_cache` executable and checked against source/voxel/checksums. Do not start three uncached raw-map loaders on a memory-constrained laptop.

Go2 height 0.5 m and tilt 20 degrees are metadata; LiDAR-to-chassis extrinsics and field navigation have not been validated. Unit/lifecycle tests and isolated recorded-data checks do not establish Go2 accuracy. `mid360_robot.yaml` includes `preprocess.max_range=5.0` for the Sensor FAST_LIO patch; this preserves the existing 5 m acquisition range.
