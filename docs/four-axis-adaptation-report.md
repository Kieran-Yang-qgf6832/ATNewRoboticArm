# 四自由度适配复查与验证报告

日期：2026-10-19
范围：`src/` 下控制、任务、仿真、辨识、RViz 与文档。约束：未修改实机安全相关控制参数
（`data_pack.h` / `ArmRealInterfaces` / `default_kp` / `default_kd` 均保持原样）。

## 1. 结论

- 控制器/任务/仿真链路已经是 4 轴：URDF/MJCF 各 4 关节，`joints` 4 项，
  `end_effector_frame=link4`，`task_space_dof=4`，`tasks.yaml` 目标点 4 维。
- `tasks.yaml` 全部 10 条序列已用 4 轴模型跑过：7 条正例退出码 0，
  3 条反例退出码 1（按设计在下发前拒绝）。
- 导纳/示教专项验收：`--mode force` 2/2、`--mode admittance` 9/9、`--mode teach` 2/2 通过。
- 辨识链路的 6 轴配置已修好，`identify_arm` 默认配置可以成功写 URDF；
  但 4 轴测量数据质量仍不足，**辨识出的 URDF 不可用**（见 `docs/bugs.md` O15）。
- 实机链路仍按 6 电机 MCU 帧，4 关节 URDF 会被 `ArmRealInterfaces::on_init` 拒绝；
  在确认硬件电机数前没有改动（安全相关）。

## 2. 关键修改

| 文件 | 修改 |
| --- | --- |
| `src/parameter_identify/config/identify.yaml` | `active_joints` 改为 `joint1..joint4`；`inertia_eig_tol` 放宽到 `-1.0e-8`（吸收 LMI 投影的 ~3e-9 求解器噪声）；去掉 6 轴 link5 的当前真值表述 |
| `src/parameter_identify/tools/verify_torque.py` | `--dof` 默认改为从 URDF 的 `model.nv` 自动读取（4 轴 4、6 轴 6），不再写死 6 |
| `src/arm_controller/controller/src/controller.cpp` | `end_effector_frame` 默认值 `link6` -> `link4`（YAML 仍显式设置 `link4`；换回 6 轴时由 YAML 覆盖） |
| `src/parameter_identify/parameter_identify/identify_arm.py` | 删除报错文案里 6 轴 `link5` 伪惯量数值，改为通用说明 |
| `src/launch_pack/tools/admittance_demo_check.py` | 修力估计复核的采样时序：外力服务会阻塞到 duration 结束，且 `~/admittance_estimate` 只有 2 Hz；改为 0.5 s 分段、段内采样 |
| `src/launch_pack/rviz/display_config.rviz` | RobotModel/TF 里陈旧的四足 link 树替换为 `world -> base_link -> link1..link4` |
| `src/launch_pack/rviz/arm_display_config.rviz` | `/robot_description` -> `/arm_robot_description`（与 O4 的话题重映射一致） |
| `README.md` / `AGENT.md` / `CLAUDE.md` / `src/arm_controller/package.xml` / `src/parameter_identify/README.md` / `docs/teach/NOTES.md` | 4 轴现状、工具新用法、默认 frame、O8 已修等陈旧描述 |
| `docs/bugs.md` | 新增 O15：4 轴辨识配置残留 + 高加速度数据质量待办 |

## 3. 实验步骤

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 构建
colcon build --symlink-install --packages-select arm_controller parameter_identify tasks launch_pack arm_model

# 1) 起 4 轴 headless 仿真
ros2 launch launch_pack arm_controller_test_sim.launch.py headless:=true show_rviz:=false

# 2) 逐条跑任务（另一终端）
for s in reset_only reset_then_joint ready_then_cart pick_demo cart_cycle_large \
         reset_then_cart unreachable_near unreachable_far admittance_demo teach_demo; do
  ros2 run tasks task_runner --ros-args -p task_sequence:=$s -p exit_when_idle:=true -p use_sim_time:=true
  echo "$s rc=$?"
done

# 3) 导纳/示教一条命令验收（脚本自己起停仿真）
python3 src/launch_pack/tools/admittance_demo_check.py --mode force
python3 src/launch_pack/tools/admittance_demo_check.py --mode admittance
python3 src/launch_pack/tools/admittance_demo_check.py --mode teach

# 4) 一条命令仿真 + 任务
ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=reset_then_joint headless:=true show_rviz:=false

# 5) 参数辨识（示例用 20 s 轨迹；默认 10 s 也能跑）
ros2 launch launch_pack arm_measure_sim.launch.py headless:=true \
  csv_file_path:=/tmp/measured_4dof_20s.csv trajectory_period:=20.0 trajectory_seed:=1
ros2 run parameter_identify identify_arm \
  --csv /tmp/measured_4dof_20s.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/identified_4dof_20s.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/identified_4dof_20s_report.yaml
python3 src/parameter_identify/tools/verify_torque.py \
  --urdf /tmp/identified_4dof_20s.urdf \
  --reference src/arm_model/model/robotic_arm.urdf \
  --csv /tmp/measured_4dof_20s.csv
```

## 4. 验证结果

### 4.1 任务序列（4 轴，headless）

| 序列 | 退出码 | 结果 |
| --- | --- | --- |
| `reset_only` | 0 | 复位 -> idel |
| `reset_then_joint` | 0 | 关节跟踪误差 0.0000 rad |
| `ready_then_cart` | 0 | 位置残余 0.0002 m |
| `pick_demo` | 0 | 位置残余 0.0007 m |
| `cart_cycle_large` | 0 | 位置残余 0.0002 m |
| `reset_then_cart` | 1（预期） | 下发前拒绝，状态回 idel |
| `unreachable_near` | 1（预期） | 下发前拒绝 |
| `unreachable_far` | 1（预期） | 下发前拒绝 |
| `admittance_demo` | 0 | 进入/保持/退出导纳正常 |
| `teach_demo` | 0 | 进入/保持/退出示教正常 |

### 4.2 导纳/示教专项

| 模式 | 结果 |
| --- | --- |
| `force` | 2/2：+x 5 N 估计 [4.429, 0.021, -0.253] N，误差 0.625 N，cos 0.998；工具轴 1 N·m 估计 0.796 N·m，误差 0.204 N·m |
| `admittance` | 9/9：x/y/z 六方向让开、限速、撤力停住、自转 ± 均通过 |
| `teach` | 2/2：+x 拖 0.2644 m / +z 拖 0.1410 m，松手漂移 3.1/4.8 mm |

### 4.3 辨识链路

- 5 s / 10 s / 20 s / 40 s 测量分别得到 2500 / 5000 / 9993 / 19982 行，measure 状态均正常结束。
- 20 s 数据上真值 URDF 的 RNEA 残差：总体 **5.58%**；低加速度一半 **4e-7**，
  高加速度一半 **11.2%**。40 s 数据总体 **6.49%**。
- 默认配置 `identify_arm` 在 20 s 数据上成功：base 参数 22 个，rmse 0.373，
  corr 0.985，`physical_validation.passed: true`。
- 但 `verify_torque.py` 显示辨识 URDF 对 CSV 的误差是**百万百分比**，
  `reconstruction_base_residual_after_projection` 也很大 —— **该 URDF 不可用于替换模型**。
  根因与待办见 `docs/bugs.md` O15。

## 5. 剩余问题（未伪装成已完成）

1. **实机 6 电机协议**：`data_pack.h` 固定 `motor[6]`，`ArmRealInterfaces` 会拒绝 4 关节
   URDF。若实机确为 4 电机，需要同步改协议、接口与 `arm_real_joint_count_test`；
   这属于安全/硬件相关，未擅自修改。
2. **辨识精度**：4 轴 joint3 惯量小，高加速度段 `qacc` 与 `τ_act` 的时序/激励带宽
   需要进一步处理；修好前不要把辨识 URDF 当先验（C7/O15）。
3. **6 轴遗留的潜伏默认值**：`measure_end_effector_tool_offset: 0.07` 是 link6 实测值，
   当前因 `measure_end_effector_z_lower_limit: -inf` 不生效；若重新启用末端 z 约束，
   需要按 4 轴工具网格重新测量。
4. **历史日志**：仓库里跟踪的 `error` / `mujoco.log` / `task_*.log` 仍含 joint5/6，
   属于历史记录，未删除。
5. **有意的 6 轴兼容层**：`Default6DofTaskSpaceMapping`、`task_space_dof=6`、
   `servo`（4 维下显式拒绝）、离线工具的 6 轴分支、任务层 6 维位姿消息契约，均保留。
