---
id: O14
title: best-effort 订阅丢帧，深度过期让分面退化为单面【高】（已修）
severity: 高
status: 已修
date: 2026-11-09 20:04
commit: 5cac0bc
---

# O14 best-effort 订阅丢帧，深度过期让分面退化为单面【高】（已修）

> 严重度：高 ｜ 状态：已修 ｜ 提交：`5cac0bc` ｜ 日期：2026-11-09 20:04

**现象**

检测一直输出 `mode=single (1)`、`issues=[depth-stale]`、`has_box=false`，
`check_sim_vision` 报「无箱体 339 帧」。但同时抓 `/vision_camera/color` 与
`/vision_camera/depth` 的 `header.stamp`，**差值是 0.000 s**；两个话题各自
`ros2 topic hz` 也有 8.7~11.8 Hz。也就是说深度并不旧，是检测节点**收不到**。

**根因**

`camera_plugin.cpp` 用 `create_publisher<Image>(topic, 1)`——reliable + `KEEP_LAST(1)`；
检测节点用的是 `qos_profile_sensor_data`（best-effort）。同一话题 4 s 计数实测：

| 订阅 QoS | 收到频率 |
| --- | --- |
| reliable（depth 10） | **14.5 Hz** |
| best-effort（depth 10） | 6.9 Hz |
| `qos_profile_sensor_data` | **4.7 Hz** |

检测主循环（19 Hz）比图像快，读到的最新深度帧可能已经落后 0.5 s 以上，
`usable_depth()` 判 `DEPTH_STALE`，`split_faces()` 整段被跳过 → 只剩单面 PnP，
`has_box` 自然为 false。

**修法**

`detector.py` 增加 `image_qos` 参数（`sensor` 默认，与 RealSense 的 best-effort 发布一致；
`reliable` 用于仿真）。`view_sim.launch.py` 传 `image_qos: reliable` 与发布端匹配。
实机路径 `view.launch.py` 不动，仍用 `sensor`。

**验证**

`mode=dual 30/30`、`issues=[]`、`has_box=true`，验收 PASS：

| 指标 | 实测 | 允许 |
| --- | --- | --- |
| 面心误差 | 1.37 mm | 20 mm |
| 法线夹角 | 0.11° | 3° |
| 箱心误差 | 1.17 mm | 15 mm |
| 边长误差 | 0.70 / 11.87 / 0.51 mm | 15 mm |

**教训**：QoS 不是"订阅端越宽松越通用"。发布端 `depth=1` 时 best-effort 读者会丢掉
约 2/3 的帧；跨进程链路要么把发布端改成 `SensorDataQoS`，要么订阅端显式匹配，
**不能靠"看起来兼容"**。

## 相关条目

- [O13](../2026-11-09/2004-仿真检测节点被clock拖慢.md)
