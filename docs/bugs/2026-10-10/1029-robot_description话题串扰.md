---
id: O4
title: launch 经 `/robot_description` 话题取 URDF，易静默加载错模型【中】
severity: 中
status: 已修
date: 2026-10-10 10:29
commit: ac474f0
---

# O4 launch 经 `/robot_description` 话题取 URDF，易静默加载错模型【中】

> 严重度：中 ｜ 状态：已修 ｜ 提交：`ac474f0` ｜ 日期：2026-10-10 10:29

**现象**

`ros2_control_node` 是从 **`/robot_description` 话题**取 URDF 的，不是用自己的参数：

```
[controller_manager]: Subscribing to '/robot_description' topic for robot description.
[controller_manager]: Received robot description from topic.
```

同 ROS domain 内只要有**任何**其他 `robot_state_publisher`（实测遇到另一个项目的残留进程），controller_manager 就会静默加载那个机器人的模型，所有控制器加载失败，而报错信息完全指不到真正原因。

**复现与补充实测**（`ac474f0` 之前）

先起一个发布 1 关节诱饵模型的 transient_local 发布者，再起本 launch：

```
[controller_manager]: Subscribing to '/robot_description' topic for robot description.
[controller_manager]: Received robot description from topic.
[controller_manager]: Received robot description from topic.          ← 收到两条
[ERROR] [controller_manager]: Exception caught while loading and initializing components:
                                  no 'ros2_control' tag found in the URDF   ← 诱饵导致的
```

两条额外结论：

1. **把 `robot_description` 作为参数传给 controller_manager 没有用**——实测它照样
   订阅话题。参数不能作为防护。
2. **初始化完成之后再收到外来描述会被忽略**，只打一条 WARN：

   ```
   [WARN] [controller_manager]: ResourceManager has already loaded a urdf and is initialized.
                                Ignoring attempt to reload a robot description.
   ```

   所以风险窗口只在**启动阶段**，但正是启动阶段决定了加载哪个模型。另外「最后一条
   描述获胜」意味着谁能抢到最后一条谁就赢——这是一场竞态。

**修复**（`ac474f0`）

把**发布端**（`robot_state_publisher`）与**订阅端**（`controller_manager`）一起重映射到
本工作区专用话题 `arm_robot_description`，从结构上杜绝串扰，而不是靠事后检测。
RViz 的 RobotModel 显示同步指向新话题。三个会拉起 controller_manager 的 launch
（`arm_controller_test_sim` / `arm_measure_sim` / `arm_mujoco_sim`）全部同步。

> 话题重映射不影响同名参数，`robot_state_publisher` 仍从 `robot_description`
> **参数**读模型，只是发布到新话题名。

**验证**（诱饵发布者全程在场）

| 检查项 | 修复前 | 修复后 |
| --- | --- | --- |
| CM 订阅的话题 | `/robot_description` | `/arm_robot_description` |
| CM 收到的描述条数 | 2（含诱饵） | **1**（只有本机的） |
| 诱饵导致的错误 | 1 条异常 | **0** |

功能回归：`reset_only` / `reset_then_joint` / `ready_then_cart` / `pick_demo`
退出码均为 0，`unreachable_far` 按预期退出码 1。

**遗留**

- 话题名不再是标准名，外部工具（如 `ros2 topic echo /robot_description`、其他
  依赖标准名的可视化工具）需要改用 `/arm_robot_description`。本工作区内除
  controller_manager 外没有订阅者，RViz 配置已同步。
- `arm_real.launch.py` / `arm_static_display.launch.py` 未同步——它们引用了不存在的包，
  当前不可用（见 CLAUDE.md）。

---
