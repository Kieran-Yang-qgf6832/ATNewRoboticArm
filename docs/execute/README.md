# 执行说明

> 环境依赖、安装构建、仿真/实机运行、标定、任务层、参数辨识与排错。
> 本目录是**操作入口**；设计原因与约束见 [ADR](../adr/README.md)，问题记录见 [缺陷索引](../bugs/README.md)。

## 阅读顺序

| 文档 | 内容 |
| --- | --- |
| [01 环境依赖与构建](01-环境依赖与构建.md) | ROS 2 / 依赖 / `colcon build` / 常见构建问题 |
| [02 仿真运行](02-仿真运行.md) | MuJoCo 仿真、一条龙 launch、headless |
| [03 视觉链路](03-视觉链路.md) | 深度相机、红色目标检测、`check_sim_vision` |
| [04 实机首次运行与标定](04-实机首次运行与标定.md) | 上电、USB 权限、loopback、关节标定、断链验证 |
| [05 任务层与序列](05-任务层与序列.md) | `tasks.yaml`、步骤类型、序列、real-compat 验收 |
| [06 参数辨识](06-参数辨识.md) | 激励轨迹、测量 CSV、离线辨识、结果验证 |
| [07 离线工具与排错](07-离线工具与排错.md) | `fk_dump` / `ik_solve` / 验收工具 / 常见报错 |
| [08 开发约定](08-开发约定.md) | 分层、实时约束、参数声明、文档维护 |

## 最短路径

```bash
# 构建
colcon build --symlink-install
source install/setup.bash

# 仿真（推荐入口）
ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pos_move_demo

# 实机前先读
# docs/execute/04-实机首次运行与标定.md
```
