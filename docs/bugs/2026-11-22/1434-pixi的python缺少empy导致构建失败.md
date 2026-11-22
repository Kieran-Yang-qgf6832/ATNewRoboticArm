---
id: O23
title: 构建因 pixi 的 python3 抢占 /usr/bin/python3、缺 empy 而失败
severity: 中
status: 已修
date: 2026-11-22 14:34
commit: —
---

# O23 构建因 pixi 的 python3 抢占 /usr/bin/python3、缺 empy 而失败

> 严重度：中 ｜ 状态：已修 ｜ 日期：2026-11-22 14:34

> 触发场景：`colcon build` 时 shell 的 `PATH` 里 `/home/kangy/.pixi/bin` 排在 `/usr/bin` 之前
> （`~/.bashrc:136` 的 `export PATH="/home/kangy/.pixi/bin:$PATH"`），且本轮有 rosidl 接口包
> 需要重新 configure。本次由 `src/easy_handeye2` 子模块改 `package.xml` 触发
> `easy_handeye2_msgs` 重新跑 CMake。

## 现象

`easy_handeye2_msgs` 在 `rosidl_generate_interfaces` → `rosidl_adapt_interfaces` 阶段直接失败，
同批并行构建的 `arm_controller` / `tasks` / `parameter_identify` 等被 SIGINT 连带中断，
表现为「整包编译失败」：

```
CMake Error at /opt/ros/jazzy/share/rosidl_adapter/cmake/rosidl_adapt_interfaces.cmake:57 (message):
  execute_process(/home/kangy/.pixi/bin/python3 -m rosidl_adapter ... --package-name easy_handeye2_msgs ...)
  returned error code 1:

  File "/opt/ros/jazzy/lib/python3.12/site-packages/rosidl_adapter/resource/__init__.py", line 19, in <module>
    import em
ModuleNotFoundError: No module named 'em'
```

其余 15 个包本身编译通过，日志里只有 `tl_expected`、`setup.py` 的弃用警告，没有任何 C++ 报错。

## 根因

- CMake 的 `FindPython3` 按 `PATH` 顺序取解释器，选中的是 pixi `science` 环境的
  `/home/kangy/.pixi/bin/python3`（3.12.16），并把结果固化进缓存：
  `build/easy_handeye2_msgs/CMakeCache.txt`：
  `_Python3_EXECUTABLE:INTERNAL=/home/kangy/.pixi/bin/python3`、
  `_Python3_INTERPRETER_PROPERTIES:...;/home/kangy/.pixi/envs/science/lib/python3.12/site-packages;...`。
- `rosidl_adapter` 依赖 `empy` 提供的顶层模块 `em`，Ubuntu 侧装在
  `/usr/lib/python3/dist-packages/em.py`（`python3-empy`），pixi 环境里没有这个包。
- 容易误判的点：`robot_msgs`、`mujoco_ros2_control_msgs` 等同样用 rosidl 的包本轮不报错，
  只是因为它们的 CMake 缓存已存在、没有重新 configure；一旦新建包或改动 `package.xml`
  触发重配置，问题就会在任何接口包上复现。
- 只改 `PATH` 而不动缓存也无效：解释器路径已被 `CMakeCache.txt` 记住，必须重新 configure
  或显式覆盖 `Python3_EXECUTABLE`。

## 修复

构建时不使用 pixi 的 bin 目录，并显式指定系统解释器：

```bash
source /opt/ros/jazzy/setup.bash
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v '/\.pixi/' | paste -sd:)
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

一次性根治（若 pixi 只是偶尔使用）可把 `~/.bashrc:136` 由前置改为后置：
`export PATH="$PATH:/home/kangy/.pixi/bin"`，让 ROS 始终命中 `/usr/bin/python3`。

## 验证

- `colcon build --packages-select easy_handeye2_msgs --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`
  → `Finished <<< easy_handeye2_msgs [25.4s]`，`CMakeCache.txt` 中 `_Python3_EXECUTABLE:INTERNAL`
  已变为 `/usr/bin/python3`。
- 随后全量 `colcon build` → `Summary: 16 packages finished [49.2s]`，
  5 个包的 stderr 仅剩 `tl_expected` 弃用与 `setup.py` 弃用警告。
- `grep -rn pixi build/*/CMakeCache.txt` 仅命中改造前的 `easy_handeye2_msgs`，其余包缓存未受影响。
