---
id: F12
title: `reset` 永不完成、FSM 卡死【严重】（原 bugs.md A1，重构前已修）
severity: 严重
status: 已修（重构前）
date: 2026-10-07 08:51
commit: —
---

# F12 `reset` 永不完成、FSM 卡死【严重】（原 bugs.md A1，重构前已修）

> 严重度：严重 ｜ 状态：已修（重构前） ｜ 提交：`—` ｜ 日期：2026-10-07 08:51

**根因**

纯 PD 无积分，joint4 的重力静差 `τ/kp = 1.0465/10 = 0.1047 rad` 远超 `reset_tolerance = 0.01`，`reset_done_` 永远为 false，`check_switch()` 只返回自身，FSM 永久卡在 `reset` 并静默吞掉所有 `exp_state` 请求。

**修复**（本轮之前完成）

`ResetState::run()` 增加兜底：`reset_duration + reset_settle` 后仍未到位则打 WARN 并强制切回 `idel`。

---
