---
id: O9
title: measure launch 的覆盖参数全部静默失效【中】
severity: 中
status: 已修
date: 2026-10-12 19:22
commit: 00001b8
---

# O9 measure launch 的覆盖参数全部静默失效【中】

> 严重度：中 ｜ 状态：已修 ｜ 提交：`00001b8` ｜ 日期：2026-10-12 19:22

**现象**：下面这条命令**看起来一切正常**——launch 起得来、测量跑得完、退出码 0——
但 CSV 仍然落在 `ros2_controller.yaml` 的默认路径，测量时长仍然是默认的 10 s：

```bash
ros2 launch launch_pack arm_measure_sim.launch.py \
    csv_file_path:=/tmp/foo.csv trajectory_period:=120.0
```

**根因**：launch 把这些覆盖项拼成一个字典传给 `Node(parameters=[...])`。launch_ros 会把
字典序列化成 **node key 为 `/**`** 的 params 文件：

```yaml
/**:
  ros__parameters:
    measure_csv_file_path: /tmp/foo.csv
    measure_trajectory_period: 120.0
```

而 `controller_manager` 只把**本节点参数里 `arm_controller.*` 前缀**的项转给控制器
（`ros2_controller.yaml` 用的正是 `arm_controller:` + `ros__parameters:` 这种显式 node key）。
`/**` 下的裸参数只会留在 `controller_manager` 自己身上——`ros2 param dump /controller_manager`
**能看到**它们，控制器却读不到，于是 `auto_declare` 取默认值，**没有任何警告**。

**实测证据（2026-10-12）**

| 传入 | 生成的 params 文件 | 控制器实际生效值 |
| --- | --- | --- |
| `csv_file_path:=identify_loop2/meas_smoke.csv` | `/**` 下有该键 | 仍是 `measured_for_identification.csv`（写在进程 CWD） |
| `trajectory_period:=12.0` | `/**` 下有 `12.0` | 仍是 10.0（CSV 4998 行 ≈ 10 s） |
| 改为 node key = `arm_controller` 的补充 params 文件 | — | **6000 行 = 12 s，CSV 落在指定路径** ✔ |

**修法**：launch 改用 `OpaqueFunction`，在启动时生成一份 node key 为 `arm_controller` 的补充
params 文件，放在 `ParameterFile(ros2_controller.yaml)` **之后**（同名参数后者优先）。
见 `src/launch_pack/launch/arm_measure_sim.launch.py`。

**为什么值得单独记一条**：这是 C6 / 教训 12 的又一实例，而且比 C6 更隐蔽——C6 是"参数没声明"，
这次**参数声明了、YAML 里也写了、params 文件内容也完全正确**，错的只是 params 文件的 node key。
只检查"我传进去了"永远发现不了，必须查"目标节点读到了什么"（`ros2 param get`）。

**状态**：已修（`00001b8`）。

---

## 相关条目

- [O10](../2026-10-13/1035-激励轨迹不可复现且欠收敛.md)
