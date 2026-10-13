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
| **加速度** | **已修（O8）**：measure 直接记录 MuJoCo 的真实 `qacc`，模型残差 31.5% → **0.37%** |

反解出的加速度满足 `‖M·a + bias − τ‖ = 1.4e-16`——记录力矩与刚体动力学在机器精度上完全一致。
所以问题只剩一个：**管线怎么得到加速度**。目前是"对 500 Hz 的记录速度做 Savitzky-Golay
平滑再求导"，换任何参数都救不回来（SG(5,3)~SG(201,5) 都是 31~34%，改用位置的二阶导也只有
20%）。而 `M(q)·q̈` 正是激励过程中力矩的主要成分，20~31% 的加速度误差足以造成基拟合 7.2% 的
残差、基参数偏离 18.5%，最终推出物理可行集。

**修法（2026-10-13 已实施，commit `O8`）**：不再"算"加速度，直接把它记下来。

1. `mujoco_ros2_control` 增加 `acceleration` 状态接口（`ControlState::qacc` →
   `MuJoCoActuatorData` / `URDFJointData` → `export_state_interfaces()`）；
2. URDF 的 6 个关节各加 `<state_interface name="acceleration"/>`；
3. `ArmController` 声明并读取该接口（步长从写死的 3 改成动态推算），存进
   `FSMArmControlFactory::state_acceleration_`；
4. `ParamterIdentify` 把它连同**仿真时间**写进 CSV：`time,pos_*,vel_*,acc_*,torque_*`。

> 次选方案（激励轨迹的**解析**加速度）实测**不可用**：跟踪误差太大，解析指令加速度与真值
> 相差 **48.8%**，比原来的数值微分（30.8%）还差。必须用仿真器里的真实 `qacc`。

**修好之后的实测（120 s 测量 / seed=20261013）**

| 指标 | 修前 | 修后 |
| --- | --- | --- |
| CSV 时间列 dt 抖动 | std 18.7%（墙钟） | **1.4%**（仿真时间） |
| 模型残差 ‖M·q̈+h−τ‖/‖τ‖ | 31.5% | **0.37%** |
| 辨识残差 rmse | 1.06 N·m | **0.018 N·m**（corr 0.999998） |
| 参数误差 ‖θ−θ_true‖/‖θ_true‖ | 11.56% | **0.39%** |
| 伪惯量最小特征值 | −0.151（非物理） | **−1.1e-10**（物理校验通过） |

**一个容易误解的点**：即使加速度修好了，SDP 仍然会判 `status=error`。这**不是数据错**——
这台机器人的真值模型本身就贴在物理可行集边界上（link5 伪惯量最小特征值只有 **6.4e-6**），
任何估计误差都会把最小二乘解推到可行集之外。所以 `identify.yaml` 默认打开
`physical_projection: true`：LMI 投影把它拉回可行集内，实测投影后与真值只差 **0.39%**，
**不会**退化成质点（那是旧数据的表现）。

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
| `headless` | `true` | **关掉 MuJoCo 自带 UI 窗口**，纯命令行跑（测量不需要界面） |
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

### 无界面（headless）运行

测量默认就是 headless——launch 会把 `headless:=true` 传给 xacro，最终写进 URDF 的
`<ros2_control><hardware>` 块，mujoco_ros2_control 收到后用 `HeadlessAdapter` 代替 GLFW，
**不开窗口、不起渲染线程**，物理线程照常以 500 Hz 推进。日志里会看到：

```text
[MujocoSimulation]: Running in HEADLESS mode.
[MujocoSimulation]: Starting the MuJoCo physics thread...
```

想临时把界面要回来看，加 `headless:=false` 即可。实测 headless 与带界面在**同一 seed 下
产生完全相同的激励轨迹**（score 2.10418 vs 2.10418），数据链路不受影响。

> 想进一步**跑得比实时快**，mujoco_ros2_control 还有个硬件参数 `sim_speed_factor`
> （`<param name="sim_speed_factor">2.0</param>` = 2 倍速）。**但现在不能直接用**：
> CSV 的 `time` 列记的是 `steady_clock` 墙钟，不是仿真时间；跑快了以后
> `csv_loader` 仍按墙钟求导，加速度会被算成错的值。要先让 measure 记录仿真时间
> （见 [docs/bugs.md O8](../../docs/bugs.md)）才能开这个开关。

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
time,pos_0..pos_5,vel_0..vel_5,acc_0..acc_5,torque_0..torque_5
```

- `time` 是**仿真时间**（秒，首样本归零），不是墙钟；
- `acc_*` 是 **MuJoCo 的真实关节加速度 `qacc`**（O8），不是对速度求导得到的。
  旧 CSV 没有这几列，`acceleration_source: recorded` 会**显式报错**而不是静默退回求导。

## 验证辨识结果（**别信报告里的 rmse**）

`identify_arm` 报告里的 `rmse` 量的是**最小二乘基参数**的拟合（`identify_arm.py:168` 的
`tau_estimated = W_base @ phi_base`），而写进 URDF 的是**物理投影之后**的完整参数。
投影一旦把参数推离基参数，报告的 rmse 就不再描述这个 URDF——实测差过 **73 倍**
（iter30：报告 0.57%，URDF 实际 41.8%）。

判断一个 URDF 能不能用，**必须拿它自己正演一遍**：

```bash
python3 src/parameter_identify/tools/verify_torque.py \
    --urdf <辨识输出.urdf> \
    --reference src/arm_model/model/robotic_arm.urdf \
    --csv <训练数据.csv> --csv <另一条没训练过的.csv>
```

工具对每份 CSV 算 `tau_pred = rnea(model, q, dq, acc)`，输出：

1. **总体 + 逐关节的相对误差**（平均/最大）；
2. **与 `--reference` 真值模型的对比**——多出的误差 ≈0 才说明辨识没引入额外误差；
3. **残差结构**：`|残差|` 与 `|dq|` / `|acc|` 的相关系数。明显正相关 = 有摩擦/阻尼
   一类**随速度变化的未建模项**。

验收标准：

| 检查 | 通过的样子 |
| --- | --- |
| 训练集误差 | 小（本项目好数据下 0.0001%） |
| **另一条轨迹的误差** | 与训练集同量级（否则是过拟合） |
| 与真值模型的差 | ≈0（否则辨识引入了额外误差） |
| 残差 vs \|dq\| 相关 | 接近 0（否则有未建模的速度相关项） |

实测（`identify_loop3/diag/`）：

| 模型 | 参数误差 | meas_04 | meas_13 | full.csv |
| --- | --- | --- | --- | --- |
| 辨识结果 | 0.0002% | 0.0001% | 0.0001% | 0.2534% |
| 真值 URDF | 0 | 0.0002% | 0.0002% | 0.2534% |
| iter30 辨识结果 | **272.7%** | 40.03% | — | 41.79% |

## 生成辨识结果

> O8 修好后这条命令**正常退出并写出物理可行的 URDF**（报告里
> `physical_validation.passed: true`）。注意报告里的 `reconstruction_status` 仍是 `error`：
> 那是原始 SDP 的结果，物理可行性由 `physical_projection` 保证——见上文。

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
- `data.acceleration_source`：默认 `recorded`，读 CSV 的 `acc_*` 列（O8 修复后的正确做法）；
  `computed` 由速度列求导，相对误差 ~31%，只在拿不到真实加速度时用。
- `data.sample_time`：**应与 CSV 实际采样周期一致**（当前配置 `0.002` = 500 Hz）。这条链路
  暂时不读它（`build_regressor_basic` / `QRDecomposer` 都不读 `ts`），但 vendored FIGAROH
  的滤波工具会读（`cutoff = ts * cut_off_frequency_butterworth / 2`），一旦启用就会静默出错。
- `reconstruction.method` / `physical_projection`：见上面的三条路线表。O8 修复后默认
  `physical_projection: true`（真值模型贴在可行集边界上，投影只差 0.39%）。
- 只想先看诊断结果（rmse / 相关性 / 参数值）时，把 `reconstruction.require_physical` 设为
  `false`，但**不要**把这样得到的 URDF 用于控制。
