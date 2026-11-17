---
id: O25
title: 残留 controller_manager 让 spawner 静默失败，arm_real.launch.py 仍启动 RViz 造成“实机在跑”的假象【高】
severity: 高
status: 已修
date: 2026-11-16 2159
commit: —
---

# O25 残留 controller_manager 让 spawner 静默失败，arm_real.launch.py 仍启动 RViz 造成“实机在跑”的假象【高】

> 严重度：高 ｜ 状态：已修 ｜ 日期：2026-11-16 2159

## 现象

`ros2 launch launch_pack arm_real.launch.py loopback:=true` 之后：

- 两个 spawner 都以退出码 1 结束；
- 但 launch 继续运行，RViz 正常显示模型，rosbag 开始录制；
- 另开 `task_runner` 跑 `joint_console_only`，下发 30° 轨迹后报
  “到位，最大关节误差 0.0000 rad”，看起来一切正常。

`arm_real.log` 里的矛盾日志：

```
[spawner_joint_state_broadcaster]: Controller already loaded, skipping load_controller
[controller_manager]: Could not configure controller ... because no controller with this name exists
[ERROR] [spawner-5]: process has died ... exit code 1
```

## 根因

两层叠加：

1. **同一 domain 里有两个 `/controller_manager`**。上一次 launch 的
   `ros2_control_node` 没有退出（只杀了 launch 脚本时子进程会脱管），它持有已激活的
   `joint_state_broadcaster` / `arm_controller`（loopback 硬件）。新 spawner 的
   `list_controllers` 请求被旧进程应答（“已加载”），`configure` 请求落到新进程
   （“没有这个控制器”），于是两个 spawner 都失败。
2. **launch 不检查 spawner 的退出码**。只有 `ros2_control_node` 自己退出才 `Shutdown`；
   spawner 失败后继续启动 RViz 与 rosbag，用户从界面完全看不出控制器没激活。

`task_runner` 的“到位、误差 0.0000 rad”来自旧进程：旧进程跑的是 `loopback=true`，
`read()` 把指令直接复制成状态，误差恒为 0，没有任何物理意义。

## 修复

`src/launch_pack/launch/arm_real.launch.py`：

1. **启动前检查**：用 `ros2 node list --no-daemon` 查询 graph，若已存在
   `/controller_manager` 就只打印一条 `[拒绝启动]` 并 `Shutdown`，不启动任何节点。
   宁可拒绝启动，也不要让用户跑在别人的控制器上。
2. **spawner 失败即停机**：`OnProcessExit` 回调里检查 `event.returncode`，
   `joint_state_broadcaster` 失败时不再 spawn `arm_controller`；任一 spawner 退出码非 0
   都打印失败原因并 `Shutdown`，避免“launch 在跑但没有控制器”。

同时把 `src/tasks/config/tasks.yaml` 的 `joint_limits` 从 6 轴旧值改成 4 轴 URDF 的实际限位
（joint1 ±90°、joint2 ±177.6°、joint3 ±179.9°、joint4 ±90°），否则 `joint_console`
会放过会撞硬限位的角度。

## 验证

1. 用一个临时同名节点冒充残留 manager（`ros2 run demo_nodes_cpp talker --ros-args
   -r __node:=controller_manager`）→ launch 只输出
   `[拒绝启动] 检测到已有 /controller_manager 在运行…`，不启动任何进程。
2. 无残留时 `loopback:=true` → `4 joints, 4 motors`、两个控制器
   `Configured and activated`、日志新增 `arm_controller 已激活`。
3. 把临时副本里的控制器名改成 `no_such_controller` → spawner 退出码 1 时打印
   `[失败] joint_state_broadcaster 没有加载/配置成功…`，`controller_manager` 收到
   `Shutdown request`，launch 结束。

## 附

`loopback=true` 下 `joint_console` 的到位误差恒为 0，不能用来判断通信是否正常；
真实链路的判据是 `/joint_states` 有非零、随姿态变化的角度，且日志里 `loopback=false`。
