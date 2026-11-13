---
id: O13
title: `use_sim_time=true` 把检测节点拖到 1 Hz【高】（已修）
severity: 高
status: 已修
date: 2026-11-09 20:04
commit: 5cac0bc
---

# O13 `use_sim_time=true` 把检测节点拖到 1 Hz【高】（已修）

> 严重度：高 ｜ 状态：已修 ｜ 提交：`5cac0bc` ｜ 日期：2026-11-09 20:04

**现象**

MuJoCo 视觉链路跑通后，检测节点的 `/target_face` 只有 **0.6~1.3 Hz**（`ros2 topic hz`
实测 0.780 / 1.298），而同一份数据离线跑 `vision.detect()` 单帧只要 **16~26 ms**
（640×480，红色 mask 55k 像素，分面模式）。检测进程 CPU 100%~110%，但主循环就是快不起来。
节点日志里深度时有时无，`mode` 在 `single` 与 `dual` 之间漂。

**根因**

`mujoco_ros2_control` 的 `clock_publisher_` 在物理循环里发 `/clock`，频率等于
`update_rate = 500 Hz`（`mujoco_simulation.cpp` 的 `publish_clock()`）。rclpy 节点开
`use_sim_time=true` 后，TimeSource 会为它订阅 `/clock`，**每条消息都走一次 Python 回调**。
实测（空节点 + `MultiThreadedExecutor(num_threads=2)`，6 s 均值）：

| `use_sim_time` | CPU |
| --- | --- |
| `true` | **106%** |
| `false` | **1%** |

executor 线程持续抢 GIL，检测主循环的墙钟时间被拉长 20~50 倍——算法本身没变，是调度被拖死。

**修法**

`detector.py` 增加 `stamp_from_image`（默认 `false`）：为 `true` 时 `_stamp_msg()` 直接返回
彩色帧 `header.stamp`，既不读节点时钟也不做漂移检查。仿真 launch 保持
`use_sim_time=false` + `stamp_from_image=true`——图像头本身就是仿真时间，这本来就是感知
节点该有的时间语义（TF 与消息都跟着传感器戳走）。

**验证**

| 项 | 修前 | 修后 |
| --- | --- | --- |
| `/target_face` 频率 | 0.6~1.3 Hz | **19 Hz** |
| 消息 `header.stamp` | 混用墙钟 / 仿真时间 | 仿真时间（实测 `sec: 235`，不是 1.79e9 的墙钟） |
| `check_sim_vision` | 超时、样本不足 | 30 帧全部 `dual`，PASS |

**教训**：rclpy 订阅 500 Hz 的 `/clock` 不是"顺手开一下"的事；带 `/clock` 的仿真里，
Python 侧节点应把传感器时间戳当唯一时间基准，别让 TimeSource 进主线程。

---

## 相关条目

- [O14](../2026-11-09/2004-best-effort订阅丢帧.md)
