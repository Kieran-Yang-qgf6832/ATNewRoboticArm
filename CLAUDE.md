# CLAUDE.md

AI 助手的工作规则。项目概况与快速开始见 [README.md](README.md)，文档职责见
[docs/README.md](docs/README.md)，设计原因见 [docs/adr/](docs/adr/README.md)，
缺陷记录见 [docs/bugs/](docs/bugs/README.md)。

## 导航

| 需要什么 | 去哪里 |
| --- | --- |
| 构建、仿真/实机运行、标定、任务层、辨识、排错 | [docs/execute/README.md](docs/execute/README.md)（按 01–09 顺序） |
| 开发约定（分层 / 实时 / 参数 / 引用规则） | [08-开发约定](docs/execute/08-开发约定.md) |
| 设计约束 C1–C10 | [ADR](docs/adr/2026-11-13/1928-设计约束C1-C10.md) |
| 缺陷的发现/分析/修复/验证 | [缺陷索引](docs/bugs/README.md) |

## 六条硬规则（改了会出事）

1. **实时路径**：`ArmController::update()` 与 FSM 的 `enter/run/check_switch/exit`
   禁止 ROS 参数读写、禁止动态内存分配。
2. **参数与接口同步**：新增控制器参数必须 `auto_declare`，否则 YAML 里的键被静默忽略（C6）；
   改控制器接口时同步检查 YAML、URDF 的 `<ros2_control>` 接口标签、插件 XML。
3. **保持命令必须锁存**：每周期重读实测值会让位置误差恒 0，手臂在重力下坠落（F1/C4）。
4. **零位形不要直接做笛卡尔运动**：那是工作空间边界，目标会被判不可达（C1）。
5. **`idel` / `Paramter` 不要顺手重命名**：前者是外部参数契约（`exp_state=idel`），
   后者是历史拼写（`lib/paramter_identify/`、`ParamterIdentify`）。
6. **实机是 5DOF，且断链≠停机**：`task_dim=5`、joint6 锁死；固件无指令看门狗（R5），
   物理急停是唯一的硬停机手段。

完整约束与理由见 [设计约束 C1–C10](docs/adr/2026-11-13/1928-设计约束C1-C10.md)。

## 引用与文档维护

- 引用缺陷**直指条目文件**（如 `docs/bugs/2026-10-13/1202-辨识加速度由速度二次求导.md`），
  不写 `docs/bugs.md O8` —— 该路径只是旧引用的兼容映射；
- 新问题写 `docs/bugs/YYYY-MM-DD/`，新决策写 `docs/adr/YYYY-MM-DD/`，有意义的变更写
  `docs/changelog/YYYY-MM-DD/`，三者都要更新各自索引；
- 操作步骤变更只改 `docs/execute/`；同一事实不要在两处写完整描述。

## 任务收尾

一次改动准备交付/验收前，对照 [文档职责](docs/README.md) 同步一次：修 bug → `docs/bugs/`；
技术决策 → `docs/adr/`；对外变化 → `docs/changelog/`；改 FSM / 参数 / launch / URDF →
`docs/execute/` 对应篇。更新后跑 `python3 docs/tools/check_docs.py`，并在回复里说明改了哪些文档。
