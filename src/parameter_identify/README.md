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

### ⚠️ 当前这份测量数据辨识不出可用模型

**症状**：`method: auto`（→ sdp）直接报 `status=error`，提示"问题不可行"。

**实测证据**（CSV = `measured_for_identification.csv`，5000 样本）：

| 项 | 值 |
| --- | --- |
| 基参数拟合 | `‖φ_base − M·θ_prior‖ = 0.365`，而 `‖φ_base‖ ≈ 1.98`（偏离先验 18.5%） |
| 问题拆解 | 只留 PSD **可行**；只留等式 `M·θ = φ_base` **可行**；两者同时 **不可行**（等式放宽到 1.0 才可行） |
| 原始辨识结果 | 明显非物理：joint3 的 `Izz = −0.153`，而 URDF 真值是 `+0.00278` |
| 投影路线结果 | 物理可行但**退化成质点**：joint3 绕质心惯量从 `1e-3` 塌到 `1e-10`（`izz` 甚至 `−1.8e-10`） |

**已排除的原因**：picos/cvxopt 缺失（已装，LP/SDP 都能解）、CAD 约束过紧（去掉后照样不可行）、
URDF 与 MJCF 模型不一致（逐杆惯量数值一致，只是主轴排列不同）、仿真里的关节阻尼/摩擦
（MJCF 没有 `damping`/`frictionloss`）。

**结论**：模型没问题，是**数据与刚体动力学假设不一致**，所以辨识出的参数被拉出物理可行集。
投影只能把结果推到可行域边界（质点），不是可用模型。建议排查方向：

1. 记录到的 `/joint_states` 或控制器 `state_[i].torque` 到底是不是"关节实际输出力矩"
   （若含 PD 修正项或与真实力矩有相位差，会直接偏置惯量项）；
2. `acceleration_source: computed` 的有限差分精度（惯量项乘的是加速度，相位误差会被放大）；
3. 激励轨迹的信息量：`measure_trajectory_period = 10 s`、`repeat_cnt = 1` 只有 10 s 数据，
   可能不足以激励全部 36 个基参数；
4. 辨识模型里 `has_friction` / `has_actuator_inertia` 都是 `false`，若实机/仿真存在这些效应
   需要打开。

在数据修好之前，`identify.yaml` 保持"失败就报错"，**不静默降级**到投影路线——那样只会得到一个
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
传给控制器）。换位置用 `csv_file_path:=/path/to/foo.csv`；只想起仿真、测量另行触发用
`run_measure:=false`，然后：

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

- `config/identify.yaml` 中的 `model.active_joints` 必须与 URDF 中参与辨识的关节名一致。
- `data.sample_time` 建议与 CSV 采样周期一致。例如 500 Hz 采样时为 `0.002`。
- `data.acceleration_source` 默认为 `computed`，会根据速度列计算加速度。
- `reconstruction.method: sdp` 需要可用的 SDP 求解依赖；如果只想先看诊断结果，可按需要调整 `reconstruction` 配置。
