# 变更：关节命令与关节状态 rosbag 记录

- 日期：2026-11-14 11:30
- 类型：功能
- 相关：[设计约束 C9](../../adr/2026-11-13/1928-设计约束C1-C10.md)、[02 仿真运行](../../execute/02-仿真运行.md)

## 背景

上位机下发给下位机的关节命令只存在于 ros2_control 的 **command interface** 里，没有任何话题可以录；
`/joint_states` 只有回报的状态。于是「实机上跑完一段运动，事后分析跟踪误差 / 通信延迟 / 复位是否到位」
只能靠控制器内部日志，无法回放。

## 变更

- `ArmController` 新增 `~/joint_command` 话题（`sensor_msgs/msg/JointState`）：
  `update()` 从**命令接口**（不是状态机缓存）拷一份快照，wall timer 以 100 Hz 发布
  `position` / `velocity` / `effort`，`name` 与 `/joint_states` 一致；
  实时线程只做定长拷贝 + `try_lock`（拿不到锁就丢本周期），不分配、不阻塞。
- 新增 `launch_pack/launch/arm_bag_record.launch.py`：`ros2 bag record`（默认 mcap）录
  `/joint_states`、`/arm_controller/joint_command`、`/arm_controller/state` 与任务层关节空间指令
  （`/arm_joint_traj`、`/arm_joint_waypoints`、`/arm_servo`）。
- `arm_real.launch.py`、`arm_controller_test_sim.launch.py` 默认拉起录制；`arm_task_sim` /
  `arm_task_vision_sim` 透传 `record_bag` 与 `bag_dir`。
- `arm_controller` 增加 `sensor_msgs` 依赖；`launch_pack` 增加 `rosbag2` / `rosbag2_storage_mcap` 的 exec 依赖。

## 影响与迁移

- 默认就录，输出在 **项目根的 `.ros/ros2bags/<arm_sim|arm_real>_<YYYYmmdd-HHMMSS>/`**
  （`.ros` 在 `.gitignore` 里，不进版本库）；`record_bag:=false` 关闭，`bag_dir` 改根目录，
  `bag_storage:=sqlite3` 换存储。
- bag 的接收时间戳是**墙钟**（不给 Python 节点开仿真时间，C9）；对齐命令与状态用 `header.stamp`。
- 命令增益（kp/kd/ki）不在 `JointState` 里，需要看 YAML；实机固件本来也不接收增益。

## 验证

- `arm_controller_test_sim.launch.py headless:=true` 跑 55 s：`ros2 bag info` 显示
  `/joint_states` 26991 条（≈490 Hz）、`/arm_controller/joint_command` 5207 条（≈94.5 Hz）、
  `/arm_controller/state` 2 条；回放抽查命令值 = 零位附近、effort = 重力前馈（joint2~4 约 −0.77 ~ −0.82 N·m）。
- `arm_real.launch.py loopback:=true` 跑 28 s：bag 1.6 MiB，`/joint_states` 2848 条、`joint_command` 2643 条；
  `record_bag:=false` 时确认不启动 rosbag2。
- `Ctrl-C` 收尾后 `metadata.yaml` 正常写入；日志中的 `exit code 2` 是 rosbag2 的 Python 收尾竞态，数据完整。

## 相关链接

- `src/arm_controller/controller/src/controller.cpp`（`cache_joint_command_snapshot` / `publish_joint_command`）
- `src/launch_pack/launch/arm_bag_record.launch.py`
