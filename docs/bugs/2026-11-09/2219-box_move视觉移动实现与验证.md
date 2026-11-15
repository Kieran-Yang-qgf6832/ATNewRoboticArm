---
id: INV10
title: box_move视觉移动实现与验证
type: investigation
date: 2026-11-09 22:19
---

# box_move视觉移动实现与验证

> 类型：investigation ｜ 日期：2026-11-09 22:19

> 2026-11-09。记录 `pos_move.orientation` 与第 12 类步骤 `box_move` 的落地与多场景实测。

### 10.1 落地要点

- 消息：`CartTrajCmd` 增加 `MODE_POSITION_AND_TOOL_AXIS=2`。控制器用
  `PoseMode::kPositionAndToolAxis`，在 `error_map()` / `jacobian_map()` 里用
  `I − z·zᵀ` 把绕工具轴的滚转投影掉：位置 3 维 + link6 z 方向 2 维，joint6 列精确为 0。
- `IKSolver` 在 5D 模式下把位置与姿态分量**分开判据**：
  `pos_move_tolerance`（米）+ `pos_move_axis_tolerance_deg`（默认 1°），不再用单一
  混合范数；预检失败日志分别报位置/工具轴残余。`set_solve_options` 增加
  `axis_tolerance` 参数，`ik_solve` 工具增加 `--axis` / `--position` 诊断模式。
- `pos_move.orientation: [x, y, z, w]`（基座系）：只取 z 轴，以起点实测姿态为基准
  构造「起点 z → 目标 z 的最小旋转」；180° 反平行时用给定四元数消歧。z 轴角误差
  > `axis_tolerance_deg` 判失败，不再告警全姿态漂移。`max_drift_deg` 在该模式下
  明确拒绝，避免“配置了没生效”。
- `box_move`：从 `base_link -> target_face` TF 取位姿，同时检查 `/target_face` 的
  `valid`、`child_frame_id` 与消息/TF 时间戳新鲜度（视觉失效后仍会广播 200 ms 旧 TF）；
  拿到一帧后快照。5D 目标位置 = 面心 − `tool_offset` × 面 z（`tool_offset` 默认
  0.039 m，link6 STL 轴向尖端），工具 z 与面 z 同向。
- 回退：控制器预检**未运动**拒绝（位姿变化 < 5 mm / 1°）时自动改发 position-only，
  目标仍是同一 link6 位置、姿态不约束；回退成功打 WARN。运动中途失败不回退，
  避免被卡住/碰撞后继续动。`box_move` 的位置残差是硬判据（回退后姿态漂移只告警）。
- 位置约束类轨迹由 `allow_constrained_traj` 控制（实机默认 `false`，2026-11-13 前由
  `real_mode` 控制）；关闭时两种位置约束轨迹都在下发前拒绝。
- 场景：新增 `scene_vision_raised.xml`（箱体/相机 z=0.6）与 `view_sim.launch.py` 的
  `scene_path` 参数；`box_pregrasp` / `box_low_pregrasp` 是演示用粗定位点。

### 10.2 实测结论

| 项 | 结果 |
| --- | --- |
| `pos_axis_demo`（保持 ready 方向 5 cm + 抬 15°） | 通过：位置 0.0009 / 0.0010 m，工具轴误差 0.04° / 0.03° |
| 抬高场景 `box_move_demo`（`box_pregrasp` + 最后 10 cm） | 通过：5D，位置残差 0.0015 m，工具轴误差 0.25° |
| 低处 `box_move_low`（原 scene_vision.xml） | 通过（回退）：5D 预检拒绝（最近可达差 8.3 mm / 6.6°），position-only 残差 0.0125 m（演示容差 0.02 m） |
| 旧 `pos_move_demo` 回归 | 通过：位置残差 0.0009 m，姿态漂移 14.43° WARN 不变 |
| 非法配置（orientation + max_drift_deg、joint_move 带 orientation） | 加载期拒绝，退出码 1 |

**5D 端点可达 ≠ 路径可达**：从 `ready` 直接走约 1 m 到抬高后的面目标，端点 IK 收敛，
但笛卡尔直线在 t≈1.6 s 越过 5D 可达集，控制器报 `ik_err=1.78e-2`、`tracking_err=0.076`
并回 `idel`；按确认的安全边界，运动已开始后不回退。演示因此先用 `joint_move` 到
`box_pregrasp` 粗定位，再由 `box_move` 走最后约 10 cm。换箱体/相机后必须重算该点。

**低处箱体是实际可达边界**：原 `scene_vision.xml` 的箱体 z≈0.175，5D 工具水平对齐
会超 joint5 限位；回退 position-only 后残差约 1.25 cm，且控制器会报 joint4 略超模型
限位（1.85 vs 1.8 rad）。所以 `box_move_low` 用 2 cm 演示容差，干净演示用抬高场景。
