# MID-360 experimental localization tools

This source bundle preserves the independently versioned `mid360_lidar` workflow (source commit `787a183` with Nav3 path adaptations). The standard `src/mid360_bringup` package and the main navigation launch remain unchanged. Use only one sensor launch per experiment.

## Experimental entry

Build Sensor and the matching Localization branch first. For sibling workspaces, set `NAV3_LOCALIZATION_WS` if its directory name differs. For a Nav3 checkout with modules, the shared `data/maps/building_split_space.pcd` is preferred. `NAV3_MAP_PCD` overrides the map explicitly.

```bash
bash tools/mid360_lidar/scripts/start_scan.sh --mode race --region space_2
```

Race runs private BBS and virtual-template validators with independent ambiguity/two-window checks. Only the authority applies the winning map-to-odom transform and publishes navigation permission. Prior tracking follows; a tracking failure starts a new race. KISS fallback remains disabled. BBS asynchronous receipt and query-wait protection are retained.

After recording closes, strict automatic library admission can create a version for the next run; the frozen 40-frame baseline is unchanged and at most 24 automatic frames are retained. Use `--no-auto-library-update` for independent comparisons. Automatic poses are pseudo-labels, not manual floor verification.

## Required data and build products

Git contains source, configuration, manifests and indices, not map PCds, reference-frame PCds/NPZ descriptors, BBS `target.bin`, recordings or build products. Transfer matching data to the same relative paths under `reference_maps/` before running. Build BBS with `/usr/bin/python3 scripts/prepare_bbs_model.py --build-only`. Exact production map caches are prepared by the matching Localization `prepare_region_map_cache` executable and checked against source/voxel/checksums. Do not start three uncached raw-map loaders on a memory-constrained laptop.

Go2 height 0.5 m and tilt 20 degrees are metadata; LiDAR-to-chassis extrinsics and field navigation have not been validated. Unit/lifecycle tests and isolated recorded-data checks do not establish Go2 accuracy. `mid360_robot.yaml` includes `preprocess.max_range=5.0` for the Sensor FAST_LIO patch; this preserves the existing 5 m acquisition range.


## space_10 initialization (2026-11-23)

```bash
bash tools/mid360_lidar/scripts/start_scan.sh --mode bbs --region space_10
bash tools/mid360_lidar/scripts/start_scan.sh --mode virtual --region space_10
```

Choose one command per run. BBS searches the region dynamically and validates competing peaks. Virtual mode retrieves up to 24 distinct hypotheses from 348 map-rendered templates at 46 provisional stations (0.5 m height, operator-reported 20-degree forward-down pitch), then uses hierarchical VGICP and the existing ambiguity/two-window guards. These templates are not measured historical keyframes and their stations are not certified traversable. The default remains space_2 race. Both space_10 modes disable the space_2 automatic library update; measured space_10 draft export is explicit and requires operator review before activation.

Transfer matching `reference_maps/bbs_space_10/target.bin` and virtual-template PCD/NPZ payloads separately; tracked manifests and indices do not contain those arrays. Rebuild the matching Localization package for dynamic-only profiles and virtual candidate validation. Keep initialization >=80%, prior >60%, input freshness, ambiguity and jump protection; there is no KISS fallback. After initialization both modes use full-map prior VGICP, while recovery searches the selected region. Two Go2 scans at one start position with different headings/routes were operator-confirmed; this is not arbitrary-start or accuracy validation. Do not assume the earlier space_2 race tests validate Go2 space_10.


## BBS＋虚拟模板竞争（2026-11-26）

`race`现按启动区域加载BBS及虚拟模板模型，支持space_1～space_13；默认区域仍space_2，不读取实采关键帧库。两路各自完成VGICP、歧义检查和两窗，唯一主节点校验版本/epoch/时效/TF/跳变后应用首个合格结果；先验跟踪期间暂停搜索，失败后重开竞争。门限、1秒冷却、无KISS、两RViz和空间2补库策略不变，其他区不自动补入space_2。补库生成的数据供显式keyframes/hierarchical模式使用。

```bash
bash /home/pc/Nav3_ws/scripts/start_scan.sh --mode race --region space_10 --check
bash /home/pc/Nav3_ws/scripts/start_scan.sh --mode race --region space_10
```

私有日志与录包来源改为`race/bbs`和`race/virtual`，权威结果仍是根目录matching_metrics.csv与/localization/status。配套Localization需重新编译以识别race_virtual_sha256参数；整轮重启后生效。隔离验证不能替代两路并行Go2实机验收。
