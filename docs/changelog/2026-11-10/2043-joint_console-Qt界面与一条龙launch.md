# 变更：joint_console Qt 界面与一条龙 launch

- 日期：2026-11-10 20:43
- 类型：功能

## 背景

终端逐项输入关节角可用但操作繁琐；仿真 + 任务需要多个终端分别启动。

## 变更

- `joint_console` 增加 `ui: true`：Qt5 窗口，6 个目标角度 + 执行时间 +
  Execute/Stop/Exit/Use current，实时显示当前角与限位；
- 窗口输入仍写入同一个 `ConsoleQueue`，限位、单步、`stop` 语义与终端版一致；
- 新增 `arm_task_sim.launch.py`（仿真 + 任务，任务结束自动关仿真）；
- 新增 `arm_task_vision_sim.launch.py`（抬高视觉场景 + 检测 + box_move_demo）；
- 修复 `task_runner` 退出时 `fflush(nullptr)` 与 `getline` 死锁，改用 `std::_Exit`。

## 影响与迁移

- Qt UI 需要 `qtbase5-dev` 与图形会话；无显示时改用终端版 `joint_console_only`；
- 用过 Qt UI 的进程在 main 末尾硬退出，`ros2 launch` 才能正常关停。

## 验证

`joint_console_ui`、`pos_move_demo`、`box_move`、`arm_task_vision_sim` 均实测退出码 0。

## 相关链接

- [任务层与序列](../../execute/05-任务层与序列.md)
- [仿真运行](../../execute/02-仿真运行.md)
