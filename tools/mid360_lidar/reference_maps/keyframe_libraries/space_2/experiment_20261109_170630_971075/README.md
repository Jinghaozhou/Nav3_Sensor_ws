# space_2入口楼梯关键帧参考数据

来源：2026-11-09 17:06新雷达实扫，28个约1秒短窗，已保守排除跨区边界和离开space_2后的采集位姿。地图版本、设备、源会话及参数见manifest.json；区域截断依据见region_selection.json；逐帧检查见verification.json。

- clouds/：XYZ二进制PCD，表达在各自末帧base_link，未重力对齐。
- keyframes.jsonl：map_T_base、map_T_odom、odom_T_base、时间跨度、点数、源定位及PCD哈希。
- review.csv：逐帧区域/楼层/平台与参考用途复核；初始未批准，不自动激活。
- 关键帧覆盖.png：绿色为保留参考帧，红线为被排除的边界及后续路线。

用户已核对本轮RViz位置与楼层一致，仍不是测量真值。区域判断采用标签表面邻域和凸包的保守前缀，非精确通行区域；仅代表已走过的入口楼梯段。原始bag完整保留，不在此复制。已完成28帧固定已有位姿的几何检查，并生成实验性Scan-Context风格描述子；未接入在线候选检索，15固定候选仍是当前启动方案。

## 2026-11-09核对与描述子状态

- 用户确认space_2入口为1层、出口为2层；space_3/4在2层，space_5为2→3层，均不扩充本库范围。
- 中间平台A/B/C仅几何暂名，逐帧平台归属仍待人工确认；位姿不是测量真值。
- descriptors_v1/含profile.json、descriptors.npz、index.jsonl、canonical_clouds/、geometry_review.csv及三张核对图。
- 28帧固定原位姿地图支持检查全部通过；30cm内最近邻比例最低97.27%，阈值内RMSE最大4.94cm，不是定位精度。
- 原始clouds/仍为未重力对齐base_link云；派生canonical_clouds/以LiDAR为中心并重力对齐，必须使用对应map_T_descriptor。
- manifest状态geometry_checked_descriptors_ready，descriptor_built=true，online_enabled=false；review.csv的use_for_reference仍false。
- 参考和查询共享5m、10cm、20环×60扇区编码；5项单测和2份同云查询入口一致性检查通过，尚无独立检索成功率。

操作约定见[地图与区域初始化手册](/home/pc/人形/项目文档/01_操作手册/地图与区域初始化.md)，证据见[描述子准备记录](/home/pc/人形/联调记录/20261109_space2描述子准备/README.md)。
