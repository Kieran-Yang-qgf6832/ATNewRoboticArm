---
id: R6
title: 实机禁用 teach / measure【已修】
severity: 中
status: 已处理
date: 2026-10-29 21:06
commit: 5a787c1
---

# R6 实机禁用 teach / measure【已修】

> 严重度：中 ｜ 状态：已处理 ｜ 提交：`5a787c1` ｜ 日期：2026-10-29 21:06

`teach_pendant` 依赖「kp = 0 + 纯阻尼」，而固件只跑自己的位置环、根本不接收 kp；
`measure` 需要关节加速度与 joint6 激励，而固件不上报加速度、joint6 无电机。
`real_mode: true` 时这两个状态在**切入前**被拒绝（`on_init` 读 YAML 与参数回调两处），
不会出现「进去了但行为不对」。

## 相关条目

- [R4](../2026-10-29/2106-实机只有5自由度.md)
- [INV8](../2026-10-29/2106-实机联调完整记录.md)
