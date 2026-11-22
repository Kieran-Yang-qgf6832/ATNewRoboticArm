# 变更：rosbag 出图增加可缩放的 HTML 交互图

- 日期：2026-11-22 10:32
- 类型：功能
- 相关：[rosbag 出图工具 plot_rosbag](../2026-11-14/1454-rosbag出图工具.md)、
  [07 离线工具与排错](../../execute/07-离线工具与排错.md)、
  [O22 状态带全为空](../../bugs/2026-11-22/1029-出图状态带全为空.md)

## 背景

`plot_rosbag` 只出 PNG：一张 12 英寸、140 dpi 的图装下整段 bag，仿真 76 s 里约 93 Hz 的
`/arm_controller/joint_command` 平均每个样本不到 1 px，实机 200 s 的 bag 更糟。想看清
10 ms 级的命令台阶、抖动与跟随时，静态图的时间精度成了瓶颈。

## 变更

- `plot_rosbag.py` 默认与 PNG 一起给每个关节输出 `<joint>.html`（plotly 交互版），
  新增 `--no-html` 关掉；其余参数（`--out-dir` / `--dpi` / `--no-feedback`）语义不变。
- 内容与 PNG 同构：四格（位置 / 速度 / 力矩 的命令 vs 反馈 + 位置误差）、四格共享 x 轴、
  `/arm_controller/state` 状态带照搬（同状态同色，首条状态之前留空）。
- **全量样本入图、用 WebGL（Scattergl）渲染、不抽稀**：放大后是逐点真值，不是拟合线。
- 交互：x 轴统一悬停（同一时刻命令与反馈并排读数）+ 竖向准星（准星画在当前子图里，
  plotly 的原生行为）+ 缩放四格同步。
- 打开时自动聚焦「命令位置变化最快」的一段（±5 s）：判据是命令位置的一阶差分，不给
  「多大算在动」定绝对阈值，也不用 `/arm_controller/state` 分段；只影响初始视角，
  `layout` 里不写 `range`，双击仍然是 plotly 的「回到初始（全时段）」。
- 同目录只落一份 `plotly.min.js`（`include_plotlyjs="directory"`），断网也能打开。
- `package.xml` 增加 `python3-plotly` 的 exec 依赖；plotly 缺失时只警告并退回 PNG。
- 顺带修复状态带一直为空的问题（见 O22）。

## 影响与迁移

- 命令不变；产物从「只有 PNG」变成「PNG + HTML」，`docs/rosbags/` 已在 `.gitignore` 里。
- 体积：76 s 仿真 bag 每关节约 2.2 MB、整目录 19 MB；558 s 实机 bag 每关节约 3 MB、
  整目录 24 MB。耗时：两个 bag 分别 5.2 s / 7.5 s（含 PNG）。
- HTML 依赖浏览器 WebGL（Scattergl）；不支持 WebGL 的环境只会显示一行提示，此时用 PNG。
- 单独拷贝某个 HTML 到别处会丢 `plotly.min.js`，要连目录一起拷。

## 验证

- `arm_sim_20261122-101142`（76 s，命令 5550 条去重后、反馈 19352 条）：6 份 PNG + 6 份 HTML。
- `arm_real_20261122-093906`（558 s，命令 55292 条、反馈 19814 条）：6 份 HTML 共 24 MB。
- 无头 Chrome（`--enable-unsafe-swiftshader`）渲染 `joint1.html`：初始 x 范围 2.753–12.753 s、
  四格联动、状态带 reset（4.9–7.8 s）/ idel（7.8 s 起）显示正常、`hovermode` 与
  `spikedistance` 已写入、`layout` 内无写死的 `range`。
- `--no-html` 只出 PNG；`--no-feedback` 时 HTML 只画命令、误差格显示提示文字。

## 相关链接

- `src/launch_pack/tools/plot_rosbag.py`
- `src/launch_pack/launch/arm_bag_record.launch.py`
