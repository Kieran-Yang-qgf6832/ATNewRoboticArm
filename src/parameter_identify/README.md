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
| `sdp`（`auto` 在装了 picos 时选中） | 任意 | 联合最优：直接解 `M·θ = φ_base` + 逐杆伪惯量 PSD。本项目真值模型贴在可行集边界上，SDP 常报 `status=error`，默认靠 `physical_projection` 兜底 |
| `nullspace` | `false` | 只做零空间重建，**不施加物理约束**，结果通常非物理，会被 `require_physical` 拒绝 |
| `nullspace` | `true` | 先零空间重建，再用 LMI 把每个连杆投影到物理可行集；加速度修好后不会退化成质点 |

### 加速度来源（O8 已修，2026-10-13）

旧链路对 500 Hz 记录速度做二次求导得到加速度，相对误差 **31.5%**；而 `M(q)·q̈` 是激励过程
中力矩的主要成分，这个误差足以把辨识结果推出物理可行集（表现为 SDP 不可行、惯性参数非物理）。
现在 `measure` 直接记录 MuJoCo 的真实 `qacc` 与仿真时间，模型残差 31.5% → **0.37%**，
参数误差 **0.39%**。

完整排查过程、修法与逐项证据见
[O8 辨识加速度由速度二次求导](../../docs/bugs/2026-10-13/1202-辨识加速度由速度二次求导.md)；
操作步骤见 [execute/06 参数辨识](../../docs/execute/06-参数辨识.md)。

> 次选方案（用激励轨迹的**解析**加速度）实测**不可用**：跟踪误差使指令加速度与真值相差
> **48.8%**，比原来的数值微分（30.8%）还差。必须用仿真器里的真实 `qacc`。

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

测量由控制器的 `measure` 状态完成。操作步骤、launch 参数与激励轨迹日志的含义见
[execute/06 参数辨识](../../docs/execute/06-参数辨识.md)。

> `measure_csv_file_path` 由控制器在**构造阶段**读取（`ParamterMeasureState` 构造函数），
> `ros2 param set` 在控制器加载后改是**不生效**的，必须改 launch/YAML 并重启控制器（C6）。

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

命令与前置条件见 [execute/06 参数辨识](../../docs/execute/06-参数辨识.md)。本包只说明两个输出：

- `--output`：写入辨识后惯性参数的 URDF；
- `--report`：辨识过程、基参数、误差与物理一致性检查报告。

> 报告里 `reconstruction_status` 可能是 `error`（原始 SDP 的结果），只要
> `physical_validation.passed: true` 就说明投影后的参数物理可行。

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
