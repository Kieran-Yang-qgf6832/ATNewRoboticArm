# parameter_identify

`parameter_identify` 用于读取 `ParamterIdentify` 保存的测量 CSV，离线辨识机械臂动力学参数，并生成更新后的 URDF 和 YAML 报告。

本包只负责“根据 CSV 生成辨识结果”。激励轨迹生成和测量数据保存由 `arm_controller` 中的 `measure` 状态完成。

## 构建

```bash
cd <项目根目录>
colcon build --packages-select arm_controller parameter_identify --symlink-install
source install/setup.bash
```

运行时会从项目根目录加载完整 FIGAROH：

```text
third_party/figaroh-plus/src
```

## 依赖与物理一致性（重要）

辨识出的完整惯性参数要**物理可行**（伪惯量半正定、主惯量满足三角不等式），需要 SDP 能力：

```bash
pip install picos cvxopt     # 或 conda install -c conda-forge picos cvxopt
```

三条路线，`identify.yaml` 默认 `method: auto`（装了 picos 走 sdp）：

| `method` | `physical_projection` | 说明 |
| --- | --- | --- |
| `sdp`（`auto` 在装了 picos 时选中） | 任意 | 联合最优：直接解 `M·θ = φ_base` + 逐杆伪惯量 PSD。**本项目当前数据判定不可行**（见下） |
| `nullspace` | `false` | 只做零空间重建，**不施加物理约束**，结果通常非物理，会被 `require_physical` 拒绝 |
| `nullspace` | `true` | 先零空间重建，再用 LMI 把每个连杆投影到物理可行集。**能写出文件，但本项目数据下会退化**（见下） |

### ⚠️ 当前辨识不出可用模型——根因是加速度估计，**不是**测量数据

**症状**：`method: auto`（→ sdp）直接报 `status=error`，提示"问题不可行"。

**实测证据**（CSV = `measured_for_identification.csv`，5000 样本）：

| 项 | 值 |
| --- | --- |
| 基参数拟合 | `‖φ_base − M·θ_prior‖ = 0.365`，而 `‖φ_base‖ ≈ 1.98`（偏离先验 18.5%） |
| 问题拆解 | 只留 PSD **可行**；只留等式 `M·θ = φ_base` **可行**；两者同时 **不可行**（等式放宽到 1.0 才可行） |
| 原始辨识结果 | 明显非物理：joint3 的 `Izz = −0.153`，而 URDF 真值是 `+0.00278` |
| 投影路线结果 | 物理可行但**退化成质点**：joint3 绕质心惯量从 `1e-3` 塌到 `1e-10`（`izz` 甚至 `−1.8e-10`） |

### 根因：加速度估计（2026-10-11 定位）

逐个排查后的结论是——**模型没问题、力矩也没问题，唯一的问题是加速度算不准**：

| 检查 | 结果 |
| --- | --- |
| 力矩语义 | 准静态样本上"记录力矩 / URDF 重力力矩" = **1.003 / 1.006 / 1.001**，正确 |
| 接口实现 | `mujoco_system_interface.cpp` 导出的 effort = `qfrc_actuator[dof]`，就是 MuJoCo 施加的关节力 |
| 时序错位 | 力矩平移 ±6 个采样，残差平坦（0.0723→0.0790），不是时序问题 |
| 管线正确性 | 用模型正演出**完美力矩**再跑同一辨识 → 基参数残差 **5.5e-14**，管线本身正确 |
| 激励条件数 | `cond(W_base) = 138`，激励充分，不是病态 |
| 模型一致性 | URDF 与 MJCF 的逐杆质量/质心/惯量一致（<0.1%） |
| **加速度** | 由数据反解 `M(q)·a = τ − bias`，与管线估计值相差 **31.5%** |

反解出的加速度满足 `‖M·a + bias − τ‖ = 1.4e-16`——记录力矩与刚体动力学在机器精度上完全一致。
所以问题只剩一个：**管线怎么得到加速度**。目前是"对 500 Hz 的记录速度做 Savitzky-Golay
平滑再求导"，换任何参数都救不回来（SG(5,3)~SG(201,5) 都是 31~34%，改用位置的二阶导也只有
20%）。而 `M(q)·q̈` 正是激励过程中力矩的主要成分，20~31% 的加速度误差足以造成基拟合 7.2% 的
残差、基参数偏离 18.5%，最终推出物理可行集。

**修复方向**：把加速度**记录**进 CSV。CSV 格式与 `csv_loader` 已经支持 `acc_*` 列和
`acceleration_source: recorded`，缺的只是控制器侧写出来：

1. 首选 MuJoCo 的真实 `qacc`——当前系统接口只导出 position/velocity/effort，需要给
   `mujoco_ros2_control` 加状态接口（改动在第三方仓库）；
2. 次选**激励轨迹的解析加速度**：`FourierTrajectory::acceleration(t)` 已存在，给
   `ExcitationTrajectory` 加个取值接口、在 `ParamterMeasureState::run()` 里写入 `acc_*`
   即可（改动全在本仓库）。注意解析值是**指令**加速度，精度取决于跟踪质量。

修好之前，`identify.yaml` 保持"失败就报错"，**不静默降级**到投影路线——那样只会得到一个
看起来 `passed: true`、实则退化的 URDF。

其他要点：

- `require_physical: true`（默认）会在结果非物理时**拒绝写出** URDF，这是有意的保护。报错形如：

  ```text
  Refusing to write a non-physical identified URDF.
  Issues: joint1: pseudo-inertia is not positive semidefinite (min eigenvalue -0.0134), ...
  ```

- 只想看诊断数据（rmse / 相关性 / 参数值）时，可以另存一份配置把 `require_physical` 设为
  `false` 跑一遍，但**不要**把这样得到的 URDF 用于控制。
- `psd_eig_tol` 在校验时按**容差**使用（允许 `min_eig ≥ -|tol|`）：SDP/LMI 求解器只能把
  `P ≽ 0` 满足到 ~1e-8 的相对精度，不留余量的话投影明明成功也会被判非物理。真实违反量是
  1e-2 量级，仍然会被拦下。

> **vendored FIGAROH 是 0.6.0**，其重建入口不接受 `strict_base_constraints` /
> `base_residual_weight` / `shape_prior` / `tensor_trust_region` /
> `central_moment_prior`。`identify_arm` 会按实际签名裁剪这些开关、打印 WARN，并把被忽略
> 的键写进报告的 `inactive_reconstruction_options`；把 `third_party/figaroh-plus` 升级到
> 带这些开关的版本后会自动启用，不需要改代码。

## 录制 CSV

推荐一条命令完成「起仿真 + 跑测量 + 落盘」，结束后 launch 自动退出：

```bash
cd <项目根目录>
source install/setup.bash
ros2 launch launch_pack arm_measure_sim.launch.py
```

CSV 默认落在**项目根目录**的 `measured_for_identification.csv`（launch 会算出绝对路径并
传给控制器）。常用 launch 参数：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `csv_file_path` | `<项目根>/measured_for_identification.csv` | 测量 CSV 输出路径 |
| `controller_urdf_path` | `arm_model` 里的真值 URDF | **控制器内部模型**（激励轨迹生成用）；被控对象仍由 `robot_description` 决定 |
| `trajectory_period` | `10.0` | 激励轨迹周期（秒） |
| `trajectory_repeat_cnt` | `1` | 重复次数；测量时长 = `move_to_start_duration + period × repeat_cnt` |
| `move_to_start_duration` | `3.0` | 移动到激励轨迹起点的时长 |
| `trajectory_seed` | `0` | 激励轨迹 CMA-ES 随机种子：**0 = 随机（不可复现）**，非 0 = 固定可复现 |
| `step_timeout` | `30.0` | 任务层单步骤超时；测量比 `measure.yaml` 的 `duration` 长时**必须放大** |

例：用上一轮辨识的模型生成 120 s 激励轨迹：

```bash
ros2 launch launch_pack arm_measure_sim.launch.py \
    controller_urdf_path:=$PWD/identified_iter_30.urdf \
    trajectory_period:=120.0 csv_file_path:=$PWD/identify_loop2/csv/meas_01.csv \
    step_timeout:=900.0
```

> 这些参数一度**全部静默失效**（params 文件的 node key 写成了 `/**`，controller_manager
> 只转发 `arm_controller.*` 前缀的项）——见 [docs/bugs.md O9](../../docs/bugs.md)。
> 改完参数请用 `ros2 param get /arm_controller measure_trajectory_period` 或 CSV 行数
> （500 Hz 下 120 s 应约 60000 行）验证它真的生效。

### 激励轨迹的可复现与可行性（O10）

控制器会在日志里打出一行：

```text
[arm_controller] 激励轨迹就绪：可辨识参数 36 个，seed=12345，原始违例 0.000e+00 rad，
                 缩放 1.0000，最终违例 0.000e+00 rad，得分 2.1513
```

- **`seed`**：本次实际使用的种子。想重放某一次测量，把 `trajectory_seed:=` 设成这个值即可
  （同一个 URDF + 同一个种子 → 逐位相同的轨迹）。默认 0 表示"每次按当前时间随机"。
- **`最终违例`**：必须为 **0**。CMA-ES 交回的原始最优解经常仍违反关节软限位，代码会用
  二分把系数整体缩放到可行域内（可行域是以 0 为内点的凸集）；若连 q≡0 都不可行，
  生成直接判失败，不会把不可行轨迹发下去。
- **`得分`**：激励质量（越大越好），可用于比较不同种子。

> 修 O10 之前，21/21 次测量的指令轨迹都违反软限位（违例 0.119 ~ 6.675 rad），
> 把关节甩出硬限位（joint6 行程 15.4 rad、速度峰值 1064 rad/s），测量数据直接报废。
> 详见 [docs/bugs.md O10](../../docs/bugs.md)。

只想起仿真、测量另行触发用 `run_measure:=false`，然后：

```bash
ros2 run tasks task_runner --ros-args \
  -p tasks_config:=$(ros2 pkg prefix tasks)/share/tasks/config/measure.yaml \
  -p task_sequence:=measure_only -p use_sim_time:=true -p exit_when_idle:=true
```

> `measure_csv_file_path` 由控制器在**构造阶段**读取（`ParamterMeasureState` 构造函数），
> 因此 `ros2 param set` 在控制器已加载后再改是**不生效**的，必须改 launch/YAML 并重启控制器
> （见 docs/bugs.md C6）。YAML 里的默认值是相对文件名，按启动进程的工作目录解析。

`measure` 状态完成激励轨迹后，会异步保存 CSV。CSV 表头格式为：

```text
time,pos_0,pos_1,...,vel_0,vel_1,...,torque_0,torque_1,...
```

## 生成辨识结果

> ⚠️ 在当前这份 CSV 上，下面这条命令会**以退出码 1 失败**（`status=error`，SDP 判定不可行），
> 这是 O8 的表现、不是命令写错。诊断用可把 `require_physical` 设为 `false`。

```bash
cd <项目根目录>
ros2 run parameter_identify identify_arm \
  --csv measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

输出：

- `--output`：写入辨识后惯性参数的 URDF。
- `--report`：辨识过程、基参数、误差和物理一致性检查报告。

## 不安装时直接调试

```bash
cd <项目根目录>
PYTHONPATH=src/parameter_identify:$PYTHONPATH python3 -m parameter_identify.identify_arm \
  --csv measured_for_identification.csv \
  --urdf src/arm_model/model/robotic_arm.urdf \
  --output /tmp/robotic_arm_identified.urdf \
  --config src/parameter_identify/config/identify.yaml \
  --report /tmp/robotic_arm_identify_report.yaml
```

## 常见参数

- `config/identify.yaml` 的 `model.active_joints` 必须与 URDF 中参与辨识的关节名一致。
- `data.acceleration_source`：`computed` 由速度列求导（**当前精度不足，见上**）；
  `recorded` 读取 CSV 的 `acc_*` 列（**修 O8 后应改用这个**）。
- `data.sample_time`：**应与 CSV 实际采样周期一致**。当前配置写的是 `0.02`（50 Hz），而实测
  CSV 是 500 Hz（`0.002`），**差了 10 倍**。这条链路暂时不读它（`build_regressor_basic` /
  `QRDecomposer` 都不读 `ts`），但 vendored FIGAROH 的滤波工具会读
  （`cutoff = ts * cut_off_frequency_butterworth / 2`），一旦启用那条路径就会静默出错。
- `reconstruction.method` / `physical_projection`：见上面的三条路线表。
- 只想先看诊断结果（rmse / 相关性 / 参数值）时，把 `reconstruction.require_physical` 设为
  `false`，但**不要**把这样得到的 URDF 用于控制。
