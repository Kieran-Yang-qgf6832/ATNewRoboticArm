# 实机首次运行与关节标定指南

面向**第一次接触本项目的操作者**。按顺序做完 Phase 0 → Phase 6，机械臂就能在实机上安全地做受控运动，
并且关节标定（`dir` / `zero`）已经可用。

> 配套阅读：[`docs/bugs.md` §8](bugs.md) 记录了下位机联调的全部实测结论（帧协议、5 自由度、断链行为），
> 本文只讲「怎么做」，不讲「为什么」。

**总预计时间**：软件 20 min（其中构建约 3 min）＋ 标定 30~60 min（取决于摆位精度）。

---

## 目录

- [0. 一句话理解这套程序](#0-一句话理解这套程序)
- [Phase 0：上电前的机械与安全检查](#phase-0上电前的机械与安全检查)
- [Phase 1：构建工作区](#phase-1构建工作区)
- [Phase 2：USB 设备权限](#phase-2usb-设备权限)
- [Phase 3：无硬件冒烟（loopback）](#phase-3无硬件冒烟loopback)
- [Phase 4：第一次真机上电](#phase-4第一次真机上电)
- [Phase 5：关节标定](#phase-5关节标定)
- [Phase 6：标定后的小步运动验证](#phase-6标定后的小步运动验证)
- [附录 A：坐标换算与标定公式](#附录-a坐标换算与标定公式)
- [附录 B：参数与限位速查](#附录-b参数与限位速查)
- [附录 C：常见报错](#附录-c常见报错)
- [附录 D：安全红线](#附录-d安全红线)

---

## 0. 一句话理解这套程序

上位机（本仓库）通过 **USB CDC** 和下位机 MCU 通信，MCU 再驱动 5 个电机。数据流：

```
上位机  ros2_control_node ── ArmRealInterfaces ──USB(VID 0x0483 / PID 0x5741)── MCU ── 5 个电机
              │
              └─ arm_controller（状态机：idel / joint_traj / cart_traj …）
```

必须先搞清楚四件事，后面所有操作都建立在这上面：

| 事实 | 后果 |
| --- | --- |
| **实机只有 5 个自由度**：joint6 没有电机、机械锁死 | 6 维指令里 joint6 那一维被忽略；任务空间降到 5 维（位置 3 + 工具指向 2） |
| **固件只接收「位置 / 速度 / 力矩前馈」**，不接收 kp/kd/ki | YAML 里的 `default_kp` 在实机上不产生任何效果，不要指望用它调刚度 |
| **固件的角度零位和 URDF 的零位没有任何共同基准，符号也可能相反** | 必须做 Phase 5 的关节标定，否则第一次下发绝对角度会朝错误方向满力矩运动 |
| **拔 USB 线 ≠ 停机**：固件没有指令超时看门狗，会一直执行最后一帧目标 | 必须有物理急停 / 断电开关，且人在旁边 |

---

## Phase 0：上电前的机械与安全检查

**逐条确认，任一条不满足就不要上电。**

- [ ] **急停或断电开关在手边**，并且确认它真的能切断电机动力（不是只断上位机）。
- [ ] 机械臂**周围 1.5 m 内无人无物**，尤其是头顶和背后（零位姿态是竖直向上伸直，需要 1.3 m 以上净空）。
- [ ] 机械臂**底座固定**，不会被反作用力矩掀翻。
- [ ] 知道 **joint6 无电机**：不要去推它，也不要在任务里规划它的运动。
- [ ] 知道 **joint5 行程只有 ±0.61 rad（±35°）**，是最容易撞限位的一轴。
- [ ] 准备好**支撑手段**（Phase 5 需要把机械臂摆到竖直姿态后上电，建议两人配合或有支架/吊带）。
- [ ] 准备**角度测量工具**：数显角度尺 / 水平仪 / 铅垂线，用于判断「竖直」。

> 上电姿态说明：固件在**上电那一刻**把当前姿态记为零位。这条特性是 Phase 5 方案 A 的基础，
> 同时也是风险点——如果你在臂歪着的时候上电，后面所有角度都会带上这个偏移。

---

## Phase 1：构建工作区

`--symlink-install` 让 `install/` 里全部是指向 `src/` 的符号链接：**改 YAML 不用重新编译**。

```bash
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source /opt/ros/jazzy/setup.bash

# 只在第一次或依赖变化后需要清理；日常改代码不需要
rm -rf build install

# 全量构建（不要用 --packages-select 只挑几个包，原因见下方提示）
colcon build --symlink-install

source install/setup.bash
```

> **为什么不能只构建 5 个包**：`launch_pack` 依赖 `mujoco_ros2_control`，删掉 `install/` 后它会被解析到
> `/opt/ros/jazzy`，而 `/opt` 那版的 `mujoco_ros2_control_plugins` 带 `test_depend mujoco_3d_lidar`，
> 后者只存在于本工作区源码里，于是 colcon 找不到 `install/mujoco_3d_lidar/...` 直接失败。
> 全量构建能一次性消掉这个问题。

**预期结果**：最后一行是 `Summary: 13 packages finished [...]`，退出码 0。

**验证 6 个关键产物存在**（缺任何一个都说明构建没成功）：

```bash
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source install/setup.bash

ls -l build/arm_controller/joint_calibrate          # 标定工具（故意不安装）
ls -l build/arm_controller/fk_dump                  # 正运动学工具
ls -l install/arm_controller/lib/libarm_controller.so
ls -l install/launch_pack/share/launch_pack/launch/arm_real.launch.py
ls -l install/arm_controller/share/arm_controller/config/joint_calibration.yaml
ros2 pkg list | grep -E "arm_controller|launch_pack|^tasks$|robot_msgs|arm_model"
```

`ls -l` 应该显示 `-> /home/kangy/.../src/...` 的符号链接。**如果指向别的路径（例如另一个工作区），
说明安装的是别人的代码，必须重新构建**（历史踩坑记录见 `docs/bugs.md` §8.3）。

> 每个新终端都要先执行：
> ```bash
> source /opt/ros/jazzy/setup.bash && source ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/install/setup.bash
> ```

---

## Phase 2：USB 设备权限

代码用 libusb **直接**打开 USB 设备（不是 `/dev/ttyACM*`），普通用户默认没有权限。没配规则的现象是：
`ros2_control_node` 报 `Failed to open USB CDC device`，然后 **整个 launch 立刻退出**（launch 里设了 `on_exit=Shutdown`）。

```bash
sudo tee /etc/udev/rules.d/99-robotic-arm.rules > /dev/null <<'EOF'
# 机械臂下位机（STM32 USB CDC）
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5741", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

**然后一定要把 USB 线拔下来再插回去**（规则不会影响已经枚举过的设备）。

验证：

```bash
lsusb -d 0483:5741
```

- 有输出（形如 `Bus 001 Device 005: ID 0483:5741 STMicroelectronics ...`）→ 设备在，权限规则已生效。
- 无输出 → 按顺序查：① USB 线是数据线不是充电线；② 下位机固件在运行；③ 固件 PID 确实是 `0x5741`
  （ST 官方模板默认 `0x5740`，本固件改过）。

> 临时应急（不推荐长期用）：`sudo -E env "PATH=$PATH" bash -c 'source install/setup.bash && ros2 launch ...'`。
> 不要用 `sudo ros2 launch`——会丢环境变量，也会让 RViz / 日志以 root 身份写文件。

---

## Phase 3：无硬件冒烟（loopback）

**不接机械臂也能跑**：`loopback:=true` 时硬件接口不打开 USB，把上一帧指令回读成状态。

```bash
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch launch_pack arm_real.launch.py loopback:=true show_rviz:=true
```

**必须看到的日志**（在 `ros2-control` 的输出里）：

```
[arm_real_interfaces]: Configured ArmRealInterfaces for 6 joints (loopback=true, state_timeout=100 ms).
[arm_real_interfaces]:   joint1: dir=+1.000 zero=+0.000000
[arm_real_interfaces]:   joint2: dir=-1.000 zero=+0.000000
...
joint_state_broadcaster spawned, spawning arm_controller
```

**新开一个终端验证**：

```bash
source /opt/ros/jazzy/setup.bash && source ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/install/setup.bash

ros2 control list_controllers
# 期望：joint_state_broadcaster[active]  arm_controller[active]

ros2 control list_hardware_interfaces
# 期望：command interfaces 共 6 关节 × 6 项（position/velocity/effort/kp/kd/ki），
#       state interfaces 共 6 关节 × 4 项（position/velocity/effort/acceleration）

ros2 topic hz /joint_states
# 期望：average rate 稳定在 100 左右（实机 update_rate = 100 Hz）

ros2 param get /arm_controller exp_state
# 期望：String value is: idel
```

看到 `arm_controller[active]` + `/joint_states` 有稳定频率 → 冒烟通过，`Ctrl-C` 结束。

> 注意：`loopback:=true` 时读到的「状态」其实是你自己的指令，`/joint_states` 数值没有物理意义，
> **不能用来标定**。标定必须走 Phase 4 的真实链路。

---

## Phase 4：第一次真机上电

### 4.1 上电

1. 把机械臂摆到一个**安全的中间姿态**（不要停在竖直伸直位置，也不要贴着限位）。
2. 下位机上电，确认 USB 已被主机识别：`lsusb -d 0483:5741` 有输出。
3. 启动：

```bash
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch launch_pack arm_real.launch.py show_rviz:=true
```

**预期日志**：`loopback=false`、5 行 `dir=/zero=` 打印（当前是占位值）、`arm_controller[active]`。

**为什么这时候机械臂不会动**：实机配置 `config/ros2_controller_real.yaml` 里 `exp_state: idel`（仿真那份是 `reset`），
`idel` 只做一件事——锁存当前实测角度并原地保持。而且它下发的是「换算回 `q_raw` 后等于当前实测值」的指令，
所以**即使 `dir/zero` 是错的，`idel` 也只是原地不动**，这是唯一在未标定状态下安全的动作。

### 4.2 确认能看到真实角度

新开终端：

```bash
source /opt/ros/jazzy/setup.bash && source ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/install/setup.bash

ros2 topic echo /joint_states --once
```

- 有 `name:` / `position:` / `velocity:` / `effort:` 四组数据 → 通信正常。
- 完全没有数据 / 日志刷 `超过 100 ms 未收到 MCU 状态包` → USB 帧解析失败或固件没在跑，见[附录 C](#附录-c常见报错)。

### 4.3 RViz 对得上

`show_rviz:=true` 会打开 RViz 并订阅 `/joint_states`。**此时 RViz 里的模型姿态应当与实物姿态一致**
（注意：显示的是**未经标定换算**的角度，方向可能整体镜像，这正常；这里只看「模型是不是跟着动」——静止时看不出，
Phase 5 第一次点动时才是第一次真正的比对）。

> 如果外部工具需要标准话题名 `/robot_description`：本工作区把它重映射到了 **`/arm_robot_description`**，
> 用别的工具时请改成这个名字。

---

## Phase 5：关节标定

### 5.1 先理解两个参数

标定就是填这两个数（每个关节一组），写在
`src/arm_controller/config/joint_calibration.yaml`：

```yaml
joints:
  joint1: {dir:  1.0, zero: 0.0}
  joint2: {dir: -1.0, zero: 0.0}   # 注意当前文件里 joint2 的 dir 已经是 -1（占位值中的特例）
  joint3: {dir:  1.0, zero: 0.0}
  joint4: {dir:  1.0, zero: 0.0}
  joint5: {dir:  1.0, zero: 0.0}
joint6:
  fixed: 0.0
```

| 参数 | 含义 | 怎么定 |
| --- | --- | --- |
| `dir` | 符号：`+1` 或 `-1`，把固件角度方向翻到 URDF 方向 | **看 RViz 与实物是否同向**（5.3） |
| `zero` | 固件角度在「URDF 零位」处的读数（弧度） | **摆到 URDF 零位读一次数**（5.4） |

换算关系（详见[附录 A](#附录-a坐标换算与标定公式)）：

```
q_urdf = dir × (q_raw − zero)          # 读状态
q_raw  = zero + dir × q_urdf          # 下发指令
```

**URDF 零位是什么姿态**（这个必须记住，标定全靠它）：

> **整条机械臂笔直竖直向上**，末端在 base 正上方约 **1.26 m**，末端工具坐标系绕竖直轴转了 180°
> （工具 z 轴朝上、x 轴指向 base 的 −x 方向）。
> 可用 FK 工具核对：`./build/arm_controller/fk_dump install/arm_model/share/arm_model/model/robotic_arm.urdf 0 0 0 0 0 0`

> ⚠️ 「笔直朝天」这个位形同时是**奇异位形**和**工作空间边界**，只适合静止标定，
> 不要在标定完成后用它做笛卡尔运动（`docs/bugs.md` C1）。

### 5.2 标定期间的操作纪律

- 全程保持 `arm_controller` 处于 `idel`；**绝对不要**在标定完成前执行 `reset`、`reset_only`、
  `reset_then_joint`、`pick_demo` ——它们会把机械臂驱动到「竖直伸直」位置，未标定时会朝错误方向运动。
- 一次只动一个关节，步长默认 `0.05 rad ≈ 2.9°`，2 s 走完。
- 任何时候想立刻停下：**先按下急停**，然后 `Ctrl-C` 掉工具进程。
- 撤销一切状态切换、回到保持：

```bash
ros2 param set /arm_controller exp_state idel
```

### 5.3 标定 `dir`（每个关节做一次）

「让机械臂自己动一下，然后比对 RViz 模型与实物转向是否一致」——这是唯一可靠的判据。

**准备**：两个终端。终端 1 保持 Phase 4 的 launch 在跑（带 `show_rviz:=true`）；终端 2 用来执行命令。

```bash
# 终端 2
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

**逐个关节执行**（建议顺序 joint1 → joint2 → joint3 → joint4 → joint5；joint1 绕竖直轴转，最安全，先拿它练手）：

```bash
./build/arm_controller/joint_calibrate --ros-args -p joint:=1 -p step:=0.05
```

工具会打印：

```
标定预览（不动作）：joint1 当前 q_urdf=+0.123456，本步将走到 +0.173456
[1/1] joint1: +0.1235 -> +0.1735（请盯住实物方向）
       实测 q_urdf(joint1) = +0.173456
```

**此时立刻观察**：实物该关节的转动方向，和 RViz 里对应关节的转动方向

- **同向** → 该关节 `dir = +1`
- **反向** → 该关节 `dir = -1`

把结果记在纸上（5 个关节共 5 个符号）。`Ctrl-C` 会立即切回 `idel` 保持当前位置，所以每测完一个关节按一次
`Ctrl-C` 也可以，或者直接接着跑下一个关节（工具内部已切回 `idel`，可以连续调用）。

> 可选交叉验证：已知固件里 `joint[2]` 的 `inv_motor = -1`，joint2 的 `dir` 有较大可能是 `-1`，
> 但**必须以实物观察为准**——URDF 的轴向定义也会贡献一个符号。

**把 5 个 `dir` 写进 YAML**（`zero` 暂时保持原值不动）：

```bash
nano ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper/src/arm_controller/config/joint_calibration.yaml
```

**然后必须重启 launch**（`ArmRealInterfaces` 只在 `on_init` 读一次这个文件；`--symlink-install` 下改文件即时生效，
但进程不重启就看不到变化）：

```bash
# 终端 1：Ctrl-C 掉 launch，然后原样再起一次
ros2 launch launch_pack arm_real.launch.py show_rviz:=true
```

重启后核对日志里的 5 行 `dir=` 是不是你刚填的值。

### 5.4 标定 `zero`

目标只有一个：**把机械臂摆到物理零位（笔直朝天）时，`/joint_states` 的 5 个读数都接近 0**
（`|q| < 0.01 rad` 可用，`< 0.002 rad ≈ 0.1°` 理想）。

#### 方案 A：断电摆位后上电（推荐，最精确）

利用「固件在上电那一刻把当前姿态记为零位」这条特性：只要**上电时机械臂正好在 URDF 零位**，
那么固件原始角度就是 0，YAML 里 `zero` 全填 `0.0` 即可，无需读数换算。

1. `Ctrl-C` 停掉 launch，**断掉电机动力**。
2. 用手（或吊带/支架配合）把机械臂摆到 URDF 零位：**整条臂笔直竖起来**。
   - 用铅垂线 / 水平仪 / 数显角度尺判断每一节是否竖直。
   - 注意重力：断电后关节可能自由下坠，**全程托住**。
   - joint5 在零位时也应为 0（工具与第 4 节共线）。
3. **保持这个姿态上电**，然后启动：

```bash
ros2 launch launch_pack arm_real.launch.py show_rviz:=true
```

4. 新终端读一次：

```bash
ros2 topic echo /joint_states --once
```

**判读**：

| 现象 | 处理 |
| --- | --- |
| 5 个 `position` 都 ≈ 0 | ✅ 标定完成，`zero` 全填 `0.0`，跳到 [Phase 6](#phase-6标定后的小步运动验证) |
| 读数不为 0（例如 joint2 = -0.03） | 固件保留了旧的零位，或摆位有偏差 → 用下面的**通用修正公式**填进去 |

**通用修正公式**（适用于任何情况，包括非方案 A）：

```
zero_new = zero_current（YAML 里当前值） + dir × R
其中 R = 机械臂处于物理零位时，该关节的 /joint_states 读数
```

例：YAML 里 `joint2: {dir: -1.0, zero: 0.0}`，零位处读到 `R = +0.0300`，
则 `zero_new = 0.0 + (-1.0)(0.0300) = -0.0300`。

改完 → **重启 launch** → 再读一次。**反复迭代，直到零位处 5 个读数都 ≈ 0**。这一步是收敛的，
一般 2~3 轮就能到 0.002 rad 以内。

#### 方案 B：不断电，用程序点动到零位（关节掰不动时用）

断电后若关节自锁无法手动转动，就用 `joint_console` 点动。前提：**5.3 的 `dir` 已经全部填好并重启过**。

```bash
ros2 run tasks task_runner --ros-args -p task_sequence:=joint_console_only
```

按提示**逐项输入 6 个关节角（单位：度）**，一次敲一个数回车，最后输入执行时间（秒，直接回车用默认 3 s）。
实际交互长这样：

```
[joint_console> 开始输入：逐项敲 6 个关节角（度），再敲执行时间（秒）]
  [1/6] joint1 角度 (deg)，回车取消本组:
2
  joint1 = 2.0 deg (0.0349 rad)
  [2/6] joint2 角度 (deg)，回车取消本组:
5
  joint2 = 5.0 deg (0.0873 rad)
  [3/6] joint3 角度 (deg)，回车取消本组:
0
  ...
  [6/6] joint6 角度 (deg)，回车取消本组:     ← 无电机，填 0 即可
0
  [7/7] 执行时间 (s)，回车取默认 3.00:
5
```

每次只改 **2~5°**，看着实物逐步逼近「笔直朝天」。到达后读 `/joint_states` 得到 `R`，
按上面的通用公式填 `zero`、重启、再读、再微调。

> ⚠️ 点动时**不要以「输入值 0」为目标**：`zero` 还没标定时，你输入的度数并不等于「离 URDF 零位多少度」，
> 一切**以实物姿态为准**。另外每次改完 `zero` 重启后，同样的输入值会对应不同的物理位置——这是正常的，
> 收敛判据始终是「实物处于零位时读数为 0」。

其他输入约定：

- 输入途中直接回车 = 取消本组，重新输入；
- 输入 `exit` = 结束关节交互步骤；
- 运动中输入 `stop` = 立即切回 `idel` 并在当前位置保持。

> 输入会先过 `src/tasks/config/tasks.yaml` 的 `joint_limits`（度）和 `max_step_deg`（单步 90°）门禁，
> 超限只提示重输，**一个指令都不会下发**。

### 5.5 标定完成的自检清单

- [ ] `joint_calibration.yaml` 里 `joint1..joint5` 的 `dir` 都是 `+1.0` / `-1.0`，`zero` 是实测值
- [ ] 重启后日志里 5 行 `dir=/zero=` 与 YAML 一致
- [ ] 机械臂摆在零位时 `/joint_states` 5 个读数都 `< 0.01 rad`
- [ ] RViz 中的模型姿态与实物**完全一致**（位置、各连杆朝向都对得上），不再镜像
- [ ] 上面几条都满足后，`reset` 才第一次变得「安全可用」

> 建议把这份 YAML **备份到仓库外**（例如 `cp ... ~/joint_calibration.yaml.bak`）。
> 下次有人 `rm -rf install` 不会丢，但误改源码会丢。

---

## Phase 6：标定后的小步运动验证

**不要**一上来就跑 `reset_only`。`reset` 会把机械臂驱动到竖直伸直位形（行程终点 + 奇异位形），
实机上非常吓人。正确的第一步是**单关节小角度**：

```bash
# 终端 1：launch 保持运行
# 终端 2：
cd ~/MyProjects/2027ATRobotArm_LowerAndUpper/ATNewRoboticArm_Upper
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 run tasks task_runner --ros-args -p task_sequence:=joint_console_only
```

建议顺序（每步 2° 起，确认后再加到 5°、10°）：

1. **joint1 ±5°**：绕竖直轴转，重力不参与，最安全。重点确认「实物转向 == RViz 转向 == 你输入的符号」。
2. **joint5 ±3°**：行程只有 ±35°，幅度要小。
3. **joint2 / joint3 / joint4 ±3°**：这三轴要扛重力，慢慢来，注意有没有异常抖动、异响、下坠。
4. 全程观察 `/joint_states` 的 `velocity` 是否平稳、`effort` 是否远离 50 N·m 上限：

```bash
ros2 topic echo /joint_states
```

全部通过后，再考虑跑序列任务：

```bash
# 复位到零位并回到空闲（会走到竖直伸直位形！务必清空周边）
ros2 run tasks task_runner --ros-args -p task_sequence:=reset_only -p exit_when_idle:=true

# 复位后运动到 ready 点
ros2 run tasks task_runner --ros-args -p task_sequence:=reset_then_joint -p exit_when_idle:=true
```

> **不要加 `-p use_sim_time:=true`**：那是仿真链路的用法，实机会把 TF 时间戳搞乱。默认 `false` 就是对的。

### 正常收工顺序

1. `ros2 param set /arm_controller exp_state idel`（确保在保持状态）
2. `Ctrl-C` 掉 launch
3. 断电机动力

---

## 附录 A：坐标换算与标定公式

### 定义

| 符号 | 含义 |
| --- | --- |
| `q_raw` | 固件上报/接收的原始角度（弧度），已含固件内部的 `inv_motor` 与 `pos_offset` |
| `q_urdf` | URDF / 控制器 / RViz 使用的角度（弧度） |
| `dir` | 每关节符号，只能是 `+1` 或 `-1` |
| `zero` | 机械臂处于 URDF 零位时，该关节的 `q_raw` 值 |

### 换算

```
读状态（MCU → 上位机）：  q_urdf = dir × (q_raw − zero)
下发指令（上位机 → MCU）：  q_raw  = zero + dir × q_urdf
角速度、力矩同步乘 dir，保证与角度在同一套符号下（见 ArmRealInterfaces::read）
```

### 为什么 zero 的修正公式是 `zero_new = zero_current + dir × R`

在机械臂处于物理零位（`q_urdf = 0`）时，按下发公式有 `q_raw = zero_true`。
而当前上位机读到的是 `R = dir × (q_raw − zero_current) = dir × (zero_true − zero_current)`，
两边同乘 `dir`（`dir² = 1`）得 `zero_true = zero_current + dir × R`。

### joint6 的特殊处理

joint6 无电机、机械锁死。`ArmRealInterfaces::read()` 把 joint6 的状态**回写成指令值**，
让 `reset` / `joint_traj` 的 6 维到位判据自洽。代价是：**`/joint_states` 里 joint6 是镜像值，不是实测值**。
所以 `joint_calibration.yaml` 里 joint6 只记一个名义 `fixed: 0.0`。

---

## 附录 B：参数与限位速查

### 关节限位（URDF / 任务层门禁）

| 关节 | 弧度范围 | 角度范围 | 任务层单步上限 |
| --- | --- | --- | --- |
| joint1 | ±1.8 | ±103.1° | 90° |
| joint2 | ±1.5708 | ±90° | 90° |
| joint3 | ±2.9 | ±166.2° | 90° |
| joint4 | −2.0 ~ +1.8 | −114.6° ~ +103.1° | 90° |
| joint5 | ±0.61087 | **±35°** | 90° |
| joint6 | ±3.14159 | ±180° | 无电机，不用管 |

### 实机与仿真配置差异（`config/ros2_controller_real.yaml`）

| 参数 | 仿真 | 实机 | 原因 |
| --- | --- | --- | --- |
| `update_rate` | 500 Hz | **100 Hz** | `write()` 是同步 USB bulk 发送（超时 2 ms），200 Hz 时一次抖动就吃满周期 |
| `exp_state` | `reset` | **`idel`** | 首次上电只锁存当前姿态，不自动复位 |
| `task_dim` | 6 | **5** | joint6 无电机，只控位置 3 + 工具指向 2 |
| `real_mode` | false | **true** | `teach_pendant` / `measure` 在实机物理不成立，切入前就被拒绝 |
| `command_interface_prefix` | `sim_pid_controller` | **空** | 硬件接口的命令键是 `<joint>/<interface>`，不带控制器前缀 |
| `gravity_compensation` | true | **false** | 下发力矩会被固件直接叠加到速度环且无上限；模型质量只在仿真里辨识过 |
| `default_kp/kd` | 200/8 … | 40/1.6 … | 仅满足接口声明，**不上总线、不产生效果** |

> `gravity_compensation` 的打开时机：通信、标定、小步运动都确认无误之后，从 `gravity_compensation_scale: 0.1`
> 起逐档加，每档都先小步试。**跳过这一步不会影响任何功能**，只是保持精度差一些。

### 常用命令

```bash
# 切状态
ros2 param set /arm_controller exp_state idel
ros2 param set /arm_controller exp_state joint_traj
ros2 param get /arm_controller exp_state

# 看控制器 / 硬件接口
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 看角度
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states

# ⚠️ 危险：绕过任务层的关节限位门禁，直接把轨迹打进控制器。
# 仅在 joint_console 已验证过该目标、且清楚后果时使用（2 个点：起点 + 终点，时间 [0, T]）
ros2 topic pub --once /arm_joint_traj robot_msgs/msg/JointTrajCmd "{
  seconds: [0.0, 3.0],
  position: [
    {pos: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
    {pos: [0.1, 0.1, 0.0, 0.0, 0.0, 0.0]}
  ]}"
```

---

## 附录 C：常见报错

| 日志 / 现象 | 原因 | 处理 |
| --- | --- | --- |
| `Failed to open USB CDC device vid=0x0483 pid=0x5741.` 且 launch 直接退出 | udev 规则没生效 / 没重插线 / 固件 PID 不是 `0x5741` | 回 Phase 2，重插线，`lsusb -d 0483:5741` 确认 |
| `超过 100 ms 未收到 MCU 状态包，判定通信中断。` | 固件没在跑 / 帧协议不匹配 / USB 线接触不良 | 检查下位机供电与固件版本；换线；确认固件是 `2026_R2_NEW_ARM_LOWER`（帧头 `int32 0x01`，下行 78 B / 上行 77 B） |
| `URDF 声明了 N 个关节，但 MCU 通信帧固定为 6 个电机，拒绝启动。` | URDF 的 `<ros2_control>` 里关节数被改过 | 恢复 `joint1..joint6` 六个关节 |
| `关节标定文件 ... 无效，拒绝启动。` | YAML 缺 `joints` 段 / `dir` 不是 ±1 / `zero` 不是有限数 / 路径错 | 按 Phase 5.1 的模板逐项核对；`dir` 只允许 `+1.0` 或 `-1.0` |
| `未提供 joint_calibration_file，按 dir=1/zero=0 直接透传。` | URDF 里 `<param name="joint_calibration_file">` 丢了 | 检查 `src/arm_model/model/robotic_arm.urdf` 的 `hardware:=real` 段 |
| `无法把控制器切到 joint_traj` / `参数服务不可用` | `arm_controller` 没激活，或节点名不是 `arm_controller` | `ros2 control list_controllers`；用 `-p controller:=<名字>` 指定 |
| `10 s 内没有收到 /joint_states，退出` | `joint_state_broadcaster` 没起 / 话题被重映射 | `ros2 topic list | grep joint_states`；`ros2 control list_controllers` |
| 设置 `exp_state: teach_pendant` 或 `measure` 失败 | `real_mode: true` 下的**正常拒绝** | 实机不支持这两个状态，回仿真链路做参数辨识 |
| 机械臂朝**反方向**动、或一下冲到限位 | `dir` 填反了 | 立刻急停；回 Phase 5.3 重新核对 |
| 机械臂静止时角度**不是 0**，但姿态看起来对 | `zero` 没标定 | 回 Phase 5.4 |

---

## 附录 D：安全红线

1. **未完成 Phase 5 标定前，只允许 `idel`**。任何绝对角度指令（`reset` / `joint_move` / `cart_move`）
   都会在错误方向上产生运动。
2. **拔 USB ≠ 停机**。固件没有指令超时看门狗，会一直执行最后一帧目标。物理急停 / 断电开关是唯一的硬停机手段。
3. **上电瞬间的姿态决定固件零位**。不要歪着上电，除非你就是要那个姿态当零位。
4. **不要在竖直伸直位形（URDF 零位）做笛卡尔运动**：那里是奇异位形 + 工作空间边界，目标会被判不可达
   （`docs/bugs.md` C1）。
5. **`gravity_compensation` 打开前要三思**：下发力矩会被固件直接叠加到速度环输出，固件侧**没有限幅**。
   打开前先确认模型质量/惯量在实机上对得上（目前只在仿真里辨识过）。
6. **joint5 只有 ±35°**，是最容易撞硬限位的轴；joint6 无电机，不要推。
7. **不要用 `sudo` 跑整个 launch**。
8. 改完 `joint_calibration.yaml` **必须重启 launch** 才生效（`on_init` 只读一次）。

---

## 变更记录

| 日期 | 内容 |
| --- | --- |
| 2026-10-27 | 首版：基于 `arm_real.launch.py`、`ArmRealInterfaces`、`joint_calibrate` 的当前实现编写；本工作区已全量重建并确认产物为指向 `src/` 的符号链接 |
