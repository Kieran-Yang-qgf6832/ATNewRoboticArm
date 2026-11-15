# 变更：rosbag 出图工具 plot_rosbag

- 日期：2026-11-14 14:54
- 类型：功能
- 相关：[关节命令与状态 rosbag 记录](1130-关节命令与状态rosbag记录.md)、[07 离线工具与排错](../../execute/07-离线工具与排错.md)

## 背景

`arm_bag_record.launch.py` 已经把 `/joint_states`（反馈）与 `/arm_controller/joint_command`（命令）
录进 bag，但看数据只能靠 `ros2 bag play` 回放或另写脚本：没有一张图能直接回答
「这段运动里哪个关节跟得不齐、误差多大、发生在哪个状态」。

## 变更

- 新增 `src/launch_pack/tools/plot_rosbag.py`，按 `tasks/waypoint_check` 的约定
  `install(PROGRAMS … RENAME)` 安装为 `ros2 run launch_pack plot_rosbag`。
- 每个关节一张 PNG，4 个子图：位置 / 速度 / 力矩（命令 vs 反馈叠加）+ 位置误差；
  输出 `docs/rosbags/<bag 名>/joint1..joint6.png`，重跑覆盖。
- 读 `/arm_controller/state` 画状态背景带（同状态固定配色；首条状态之前留空，
  因为那段的控制器状态无法从 bag 得知）。
- bag 路径可省略，缺省取 `.ros/ros2bags` 下最新的 bag 并回显实际路径。
- 缺 `/arm_controller/joint_command` 的旧 bag 自动降级为只画反馈，标题与终端都标注。
- `.gitignore` 加 `/docs/rosbags/`（运行产物，不进版本库）。
- `launch_pack/package.xml` 增加 `rosbag2_py` / `rclpy` / `rosidl_runtime_py` /
  `python3-matplotlib` / `python3-numpy` 的 exec 依赖。

## 影响与迁移

- 命令：`ros2 run launch_pack plot_rosbag [bag] [--out-dir DIR] [--dpi N] [--no-feedback]`。
- `--no-feedback` 只画命令曲线（时间轴与关节集合仍取 `/joint_states`），位置误差子图留空并标注；
  输出文件名不变，会覆盖同一 bag 之前出的图。
- **不画加速度**：`JointState` 没有该字段、实机 MCU 也不上报，差分出来的值只能看趋势，
  因此不做（要加速度得先改录制口径）。
- 时间轴用 `header.stamp`（命令与状态同一时钟）；`/arm_controller/state` 没有 header，
  用 `/joint_states` 的「接收时间 → header 时间」线性映射换算（仿真里墙钟≠仿真时间，必须换算）。
- 命令 `effort` 是前馈力矩、反馈 `effort` 是执行器实际出力（仿真 `qfrc_actuator` / 实机 MCU 上报），
  两者不同义，不能当误差看。
- 图内中文标签；找不到 CJK 字体时自动退回英文并警告（不画方块）。

## 验证

- `arm_task_sim.launch.py task_sequence:=pos_move_demo headless:=true` 录 19 s：
  `/joint_states` 9198、`/arm_controller/joint_command` 1535、`/arm_controller/state` 7 条，
  出 6 张 PNG。状态带与实测状态序列对齐（reset 3.5 s → idel 6.5 → reset 7.0 → idel 10.0 →
  joint_traj 10.5 → cart_traj 14.0 → idel 17.0），joint4 复位段位置误差峰值约 0.29 rad（可见下垂与追赶）。
- 只含 `/joint_states` 的旧式 bag：输出警告，6 张图均为「仅反馈」，误差子图显示缺命令提示。
- `--out-dir /tmp/legacy_plots` 生效；不存在的 bag 路径报错退出。

## 相关链接

- `src/launch_pack/tools/plot_rosbag.py`
- `src/launch_pack/launch/arm_bag_record.launch.py`
