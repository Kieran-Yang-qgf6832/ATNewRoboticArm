# 变更：视觉链路与 box_move

- 日期：2026-11-09 22:19
- 类型：功能
- 相关：[视觉 ADR](../../adr/2026-11-09/2004-视觉采用HSV深度分面与PnP.md)

## 背景

需要从固定深度相机检测红色目标箱，并让机械臂按目标面位姿运动。

## 变更

- MuJoCo 深度相机接入（`scene_vision.xml`，640×480 / fovy 60 / 32FC1）；
- `view/red_quad_detector`：HSV → 四边形 → 深度分面 → PnP；
- 发布 `/target_face`（`TargetFace` 消息）与 `target_face` TF；
- 新增 `view_sim.launch.py`（解析 MJCF 相机位姿）与 `check_sim_vision`；
- 新增 `box_move` 步骤：工具 z 对齐面 z，5D 预检不可达时回退 position-only；
- 新增 `scene_vision_raised.xml`（箱体/相机 z=0.6）。

## 影响与迁移

- 仿真和 `view_sim` 必须指向同一个场景文件；
- 检测节点不开 `use_sim_time`，图像订阅用 `reliable`；
- 换箱体/相机后要重算 `box_pregrasp` / `box_pregrasp_5dof_high`。

## 验证

`box_move_demo` 实测位置残差 0.0015 m、工具轴 0.26°（6DOF）；
real-compat 5DOF 下 0.0021 m / 0.34°。

## 相关链接

- [O13](../../bugs/2026-11-09/2004-仿真检测节点被clock拖慢.md)
- [O14](../../bugs/2026-11-09/2004-best-effort订阅丢帧.md)
- [box_move 调查](../../bugs/2026-11-09/2219-box_move视觉移动实现与验证.md)
