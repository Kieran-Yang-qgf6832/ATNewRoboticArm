# Tasks

- [x] Task 1: 新增多路点消息并接入构建
  - [x] SubTask 1.1: 新增 `src/robot_msgs/msg/WaypointTrajCmd.msg`（`space`、`total_duration`、`Vector[] waypoints`，含 `SPACE_JOINT` / `SPACE_CARTESIAN` 常量）
  - [x] SubTask 1.2: 在 `src/robot_msgs/CMakeLists.txt` 注册新消息
  - [x] SubTask 1.3: `colcon build --packages-select robot_msgs`，确认生成的头文件可被引用

- [x] Task 2: 实现控制器侧 C3 规划器与离线校验工具
  - [x] SubTask 2.1: 新增 `src/arm_controller/lib/calculate/waypoint_trajectory.{hpp,cpp}`：接受 2–9 个关节节点，起止 v/a/j = 0，中间节点导数用非均匀差分估计，构造分段七次 Hermite
  - [x] SubTask 2.2: 实现自动时间分配：按单位时长采样各阶导数峰值，按 1/T、1/T²、1/T³ 缩放迭代求每段最短时长（最多 5 轮、收敛阈值 1%）；支持总时长覆盖与过短拒绝
  - [x] SubTask 2.3: 实现全轨迹采样校验：关节位置不越限，采样速度/加速度/jerk 不超限；提供 `sample(t, p, v, a, j)` 接口
  - [x] SubTask 2.4: 新增 `src/arm_controller/tools/waypoint_planner_test.cpp`，对 1..8 个用户路点（含自动起点共 2..9 节点）逐一验证：起止导数为零、内部节点 p/v/a/j 左右一致（1e-9）、限位与时长可行；构建后在 `build/arm_controller/` 下运行通过

- [x] Task 3: 控制器集成
  - [x] SubTask 3.1: `controller.cpp` 的 `on_init` 增加 `multi_traj_max_vel` / `multi_traj_max_acc` / `multi_traj_max_jerk` 的 `auto_declare`，构造期读取、长度校验（1 或关节数）、速度与 URDF `velocity` 取 min，并打一行实际生效值日志
  - [x] SubTask 3.2: `JointTrajState` 增加 `/arm_joint_waypoints` 订阅：`STOP` 阶段接受，校验 `space` 与点数，调用规划器，成功后进入 `MOVING`；`run()` 按规划曲线采样并写位置/速度/力矩前馈
  - [x] SubTask 3.3: `CartTrajState` 增加 `/arm_cart_waypoints` 订阅：同一套校验，先逐点 IK（首点用实测关节角作种子，后续用前一解作种子，四元数归一化），再转关节空间规划；`real_mode=true` 时在下发前拒绝
  - [x] SubTask 3.4: 复用现有失败路径：任一步失败写 `set_fault_stage`、`state=STOP`、由 `update()` 切回 `idel`；日志包含失败路点下标与原因
  - [x] SubTask 3.5: 确认原有 `JointTrajCmd` / `CartTrajCmd` 路径与 `STOP` 门控行为不变

- [x] Task 4: 任务层步骤类型与执行
  - [x] SubTask 4.1: `Step` 增加 `std::vector<std::vector<double>> waypoints`；`step_types.cpp` 的字段白名单增加 `waypoints`，步骤表增加 `joint_waypoints` / `cart_waypoints`（`exp_state` 分别为 `joint_traj` / `cart_traj`，`needs_duration=false`）
  - [x] SubTask 4.2: `parse_step` 解析 `waypoints`（必须是数组的数组），`validate_step` 校验 1–8 个、维度（6 / 7）、数值有限，报错含路点下标
  - [x] SubTask 4.3: `TaskContext` 新增 `publish_joint_waypoints` / `publish_cart_waypoints`（填 `space`、`total_duration`、路点）
  - [x] SubTask 4.4: `TaskExecutor` 新增两个执行函数：切状态、读取起点（关节取 `/joint_states`，笛卡尔取 TF）、下发、等待总时长、核对控制器状态与终点残差；`duration` 透传为 `total_duration`
  - [x] SubTask 4.5: 在 `step_spec` 的 `needs_duration=false` 前提下，确认 `duration` 留空时下发 0（自动）

- [x] Task 5: 新增验证序列
  - [x] SubTask 5.1: `tasks.yaml` 增加 8 条 `joint_waypoints` 序列，分别使用 1..8 个用户路点，路点均落在关节限位内且首尾姿态有可见差异
  - [x] SubTask 5.2: `tasks.yaml` 增加 8 条 `cart_waypoints` 序列，分别使用 1..8 个用户路点，路点在仿真工作空间内且四元数归一化
  - [x] SubTask 5.3: 在无视觉的主仿真场景（`scene.xml`）中逐条试跑，确认配置能加载、步骤不会因语法被拒

- [x] Task 6: 新增仿真验收工具
  - [x] SubTask 6.1: 新增 `src/tasks/tools/waypoint_check.py`：订阅 `/joint_states`，在序列执行窗口内记录数据，检查关节路点经过误差、限位、速度/加速度/jerk 有限差分峰值，退出码反映成败
  - [x] SubTask 6.2: `src/tasks/CMakeLists.txt` 安装该脚本，使其可通过 `ros2 run tasks waypoint_check` 调用
  - [x] SubTask 6.3: 用一条 1 路点关节序列做冒烟，确认工具能正确判定通过/失败

- [x] Task 7: 仿真全量验证
  - [x] SubTask 7.1: 在 MuJoCo 仿真中依次运行 8 条关节序列，每条用 `waypoint_check` 验收并记录结果
  - [x] SubTask 7.2: 在 MuJoCo 仿真中依次运行 8 条笛卡尔序列，每条用 `waypoint_check` 验收并记录结果
  - [x] SubTask 7.3: 运行离线 `waypoint_planner_test`，确认 1..8 路点全部通过导数连续性与限位校验
  - [x] SubTask 7.4: 若任一用例失败，定位是规划、IK、限位还是验收阈值问题，修复后重跑该用例与相邻路点数

- [x] Task 8: 文档更新
  - [x] SubTask 8.1: `CLAUDE.md` 补充两个新步骤类型、`waypoints` 字段、3 个新参数、`waypoint_check` 验收入口，以及“笛卡尔路点先 IK 再关节空间 C3”的说明
  - [x] SubTask 8.2: 在 `docs/bugs.md` 追加本次实现中发现的坑（如时间分配迭代、过冲、IK 种子），若无则不写

# Task Dependencies

- Task 2 依赖 Task 1（消息类型）
- Task 3 依赖 Task 2（规划器）
- Task 4 依赖 Task 1（消息）与 Task 3（控制器接口）
- Task 5 依赖 Task 4（步骤类型）
- Task 6 与 Task 5 可并行
- Task 7 依赖 Task 5、Task 6
- Task 8 依赖 Task 7
