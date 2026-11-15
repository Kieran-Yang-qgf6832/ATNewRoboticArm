# ADR-20261010-1214：任务空间采用 ModelBase + TaskMapping 分层

- 状态：已采纳
- 日期：2026-10-10 12:14（回溯整理；分层在此之前已落地）
- 相关：[O5 任务雅可比](../../bugs/2026-10-08/1728-任务雅可比在180度处失效.md)

## 背景

运动学/动力学计算既要依赖具体机器人模型（关节数、限位、惯量、几何），又要表达任务空间
语义（6 维位姿、5 维位置 + 工具指向、3 维纯位置、零空间偏好）。如果两者混在一个类里，
换模型或改任务维度都会牵动另一侧，5 自由度实机适配也会变得困难。

## 决策

采用三层结构：

```
ArmSolve → IKSolver → ModelBase + TaskMapping
```

- `ModelFromURDF`（Pinocchio）实现 `ModelBase`：FK、几何雅可比、RNEA、关节限位；
- `Default6DofTaskSpaceMapping` 实现 `TaskMapping`：任务空间语义、误差/雅可比投影；
- `ArmSolve` 负责把两者组装成逆解流程。

**约定**：修改任务空间时同步检查 `position_map()` 与 `jacobian_map()`。

## 备选方案

- **单一 Model 类同时管模型与任务空间**：实现直接，但换模型/改维度会互相影响。
- **直接使用 KDL/MoveIt 的任务空间接口**：依赖重，且难以精确表达 5 维“绕工具轴滚转自由”
  与位置约束模式。
- **每个任务维度复制一套求解器**：重复代码，容易漂移。

## 后果

- 换机器人模型只改 `ModelBase` 实现，任务空间语义保持不变；
- 6 维、5 维、3 维模式共用同一套 IK，只切换映射与判据；
- 两套映射必须同步维护，否则会出现“位置对了、雅可比错了”的隐蔽问题（O5 的教训）。

## 相关链接

- [O5 任务雅可比在 |rv|≈π 处失效](../../bugs/2026-10-08/1728-任务雅可比在180度处失效.md)
- `src/arm_controller/arm/model_from_urdf.cpp`
- `src/arm_controller/arm/default6dof_task.cpp`
- `src/arm_controller/lib/calculate/arm.cpp`
