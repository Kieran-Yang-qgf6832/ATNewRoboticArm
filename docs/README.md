# 文档体系

本项目文档分为五类，按“问题 → 决策 → 执行 → 变更”组织：

| 目录 | 记录什么 | 入口 |
| --- | --- | --- |
| `bugs/` | 缺陷的完整生命周期：发现、分析、修复、验证 | [缺陷索引](bugs/README.md) |
| `adr/` | 重要技术选型、设计原因、备选方案与最终决策 | [ADR 索引](adr/README.md) |
| `execute/` | 环境依赖、安装构建、仿真/实机运行、标定、排错 | [执行说明](execute/README.md) |
| `changelog/` | 有意义的版本、功能、兼容性变化（不是 git commit 明细） | [变更记录](changelog/README.md) |
| `teach/` | 运动学、导纳、参数辨识等教学材料 | [教学材料](teach/) |

## 命名与维护

- `bugs/`、`adr/`、`changelog/` 使用 `YYYY-MM-DD/HHMM-中文短语.md`；
- 每个分类有 `README.md` 索引，新增条目后必须更新索引；
- `execute/` 是稳定的执行文档，不按日期分目录；
- 兼容入口：旧 [`bugs.md`](bugs.md) 与 [`real_first_run.md`](real_first_run.md) 保留为重定向页；
- 运行期日志不进入文档目录（`docs/log/` 已在 `.gitignore` 中忽略）。
