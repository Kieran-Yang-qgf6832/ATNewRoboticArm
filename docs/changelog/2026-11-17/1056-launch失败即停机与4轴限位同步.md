# 变更：实机 launch 失败即停机 + 4 轴关节限位同步

- 日期：2026-11-17 10:56
- 类型：修复 / 行为变化
- 相关：[O25](../../bugs/2026-11-16/2159-spawner失败后launch继续运行.md)、
  [实机首次运行与标定](../../execute/04-实机首次运行与标定.md)

## 变化

1. `arm_real.launch.py` 启动前用 `ros2 node list` 检查是否已有 `/controller_manager`；
   有则只打印 `[拒绝启动]` 并退出，不再和残留进程抢同名服务。
2. 两个 spawner 的退出码都会被检查：`joint_state_broadcaster` 失败时不再继续 spawn
   `arm_controller`；任一 spawner 非 0 退出即 `Shutdown`，不再出现“RViz 在跑但控制器没激活”。
3. `src/tasks/config/tasks.yaml` 的 `joint_limits` 改为 4 轴 URDF 的实际限位：
   joint1 ±90°、joint2 ±177.6°、joint3 ±179.9°、joint4 ±90°。

## 兼容性

- `joint_console` 的输入门禁变严/变宽同时存在：joint1 由 ±103.1° 收紧到 ±90°，
  joint4 由 −114.6°~+103.1° 收紧到 ±90°；joint2 由 ±90° 放宽到 ±177.6°，
  joint3 由 ±166.2° 放宽到 ±179.9°。以前能通过门禁的 joint1/joint4 大角度现在会被拒绝。
- 启动流程多一次 `ros2 node list --no-daemon` 查询，正常情况下增加 1~2 s。
- 只影响实机链路与 `tasks` 的交互门禁，仿真 launch 与控制器逻辑不变。

## 验证

- 临时同名节点存在时：launch 输出 `[拒绝启动]`，不启动任何进程；
- 无残留时 `loopback:=true`：`4 joints, 4 motors`、两个控制器 `Configured and activated`、
  新增日志 `arm_controller 已激活`；
- 临时把 spawner 控制器名改成 `no_such_controller`：退出码 1 时打印 `[失败] …` 并
  `Shutdown`，launch 结束。
