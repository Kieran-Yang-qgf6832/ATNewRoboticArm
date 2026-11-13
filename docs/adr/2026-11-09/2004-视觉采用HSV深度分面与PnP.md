# ADR-20261109-2004：视觉采用 HSV + 深度分面 + PnP

- 状态：已采纳
- 日期：2026-11-09 20:04
- 相关：[O13](../../bugs/2026-11-09/2004-仿真检测节点被clock拖慢.md)、[O14](../../bugs/2026-11-09/2004-best-effort订阅丢帧.md)、[box_move 调查](../../bugs/2026-11-09/2219-box_move视觉移动实现与验证.md)

## 背景

需要从固定深度相机中检测边长 0.35 m 的红色目标箱，并给出目标面位姿供 `box_move` 使用。
目标颜色明确、场景简单，但需要同时满足仿真（MuJoCo 深度相机）与实机（RealSense）两条链路。

## 决策

检测链路固定为：

```
HSV 分割 → 四边形拟合 → 深度分面 → PnP → TargetFace + target_face TF
```

- 颜色阈值、最小面积、深度分面参数都做成节点参数；
- 仿真里检测节点**不开 `use_sim_time`**，用图像 `header.stamp` 作唯一时间基准（O13）；
- 图像订阅使用 `image_qos: reliable`（O14）；
- 相机安装位姿只在 MJCF 中定义一次，`view_sim.launch.py` 解析 `<camera pos/xyaxes>` 生成 TF。

## 备选方案

- **只做颜色阈值 + 质心**：无法给出面法线，`box_move` 的工具轴约束无法实现。
- **ArUco/二维码**：精度高，但需要在箱体上加标记，改变演示对象。
- **深度学习检测**：鲁棒性更好，但引入训练/推理依赖，当前场景不需要。
- **直接使用点云平面分割**：信息更全，但计算量更大，且仿真深度图分辨率有限。

## 后果

- 仿真验收达到面心 1.37 mm、法线 0.11°、箱心 1.17 mm；
- 检测对遮挡敏感：joint6 锁死后夹爪朝向改变，可能挡住目标面，导致 `box_move` 选错面；
  因此 real-compat 视觉验收使用从箱体上方接近的 `box_pregrasp_5dof_high`；
- 换箱体/相机位置后必须重算粗定位点；
- 实机仍使用 `view.launch.py`（RealSense），与仿真链路分离。

## 相关链接

- [O13 检测节点被 /clock 拖慢](../../bugs/2026-11-09/2004-仿真检测节点被clock拖慢.md)
- [O14 best-effort 订阅丢帧](../../bugs/2026-11-09/2004-best-effort订阅丢帧.md)
- [box_move 实现与验证](../../bugs/2026-11-09/2219-box_move视觉移动实现与验证.md)
- `src/view/`、`src/arm_model/model/scene_vision.xml`
