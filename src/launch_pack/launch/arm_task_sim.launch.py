"""
@file arm_task_sim.launch.py
@brief 任务层运行场景：一条命令完成「起仿真 → 跑 tasks.yaml 里的一条序列 → 收尾」

启动内容（按实际启动顺序）：
  1. robot_state_publisher  ：xacro 展开 arm_model/model/robotic_arm.urdf，发布
                              /arm_robot_description 与 TF（tasks 查询末端位姿依赖它；
                              话题名不是标准 /robot_description，原因见 O4）
  2. mujoco_ros2_control    ：ros2_control_node，MuJoCo 以 SystemInterface 插件运行
  3. joint_state_broadcaster：发布 /joint_states（tasks 的关节反馈来源）
  4. sim_pid_controller + arm_controller：链式 PID + 状态机控制器（一次性激活）
  5. task_runner            ：执行 tasks/config/tasks.yaml 里 task_sequence 指定的序列
                              （默认跑完即退出，on_exit=Shutdown 关掉整个 launch）
  6. rviz2                  ：由 show_rviz 控制，默认开

用法：
    # 跑默认序列（reset_then_joint：复位 → 关节运动到 ready → 回 idel）
    ros2 launch launch_pack arm_task_sim.launch.py

    # 跑指定序列（序列名见 tasks/config/tasks.yaml 的 sequences 段）
    ros2 launch launch_pack arm_task_sim.launch.py task_sequence:=pick_demo

    # 只起仿真，任务另开终端手动触发（便于反复跑不同序列、换参数）
    ros2 launch launch_pack arm_task_sim.launch.py run_task:=false
    ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:=reset_then_joint

    # 常驻任务层：序列跑完不退出，之后用 ros2 param set /task_runner task_sequence ... 换序列
    ros2 launch launch_pack arm_task_sim.launch.py exit_when_idle:=false

    # 无界面跑
    ros2 launch launch_pack arm_task_sim.launch.py headless:=true show_rviz:=false

    # 换一份任务配置文件
    ros2 launch launch_pack arm_task_sim.launch.py tasks_config:=/path/to/my_tasks.yaml

输出：任务状态发在 /task_status 话题（transient_local），内容形如
      "sequence:finished <name>" / "sequence:failed: <原因>"，也可用
      ros2 topic echo /task_status 观察。

@warning tasks.yaml 里 reset_then_cart / unreachable_near / unreachable_far 是**预期失败**
         的反例（零位形在工作空间边界 / y 方向结构不可达 / 远超工作空间），task_runner
         进程退出码为 1 是正确行为。
         当前生效模型是 **4 自由度**，控制器任务空间是 4 维 (x, y, z, joint4)：
         正例是 reset_only / reset_then_joint / ready_then_cart / pick_demo；
         cart_move 的 offset 只对 x/y/z 有意义，工具指向会随位置漂 4–8°。
         详见 tasks.yaml 顶部注释与 docs/bugs.md O13。

@note 退出码：task_runner 的退出码在 launch 日志里（`process has died ... exit code 1`），
      `ros2 launch` 自身的退出码**恒为 0**（on_exit=Shutdown 只是关掉系统，不传播子进程
      状态）。判断序列成败请看日志或订阅 /task_status 的 `sequence:finished` /
      `sequence:failed:` 消息。

@note 与 arm_measure_sim.launch.py 的分工：本文件跑 tasks.yaml 的通用任务序列，
      测量专用配置（tasks/config/measure.yaml + measure 参数覆盖）仍在
      arm_measure_sim.launch.py 里，两者互不影响。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")
    tasks_share = get_package_share_directory("tasks")

    plant_urdf_path = os.path.join(arm_model_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    # MuJoCo 插件配置（外部力注入，docs/bugs.md O14）：导纳/示教的演示要能施力。
    mujoco_plugins_yaml = os.path.join(launch_pack_share, "config", "mujoco_plugins.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")
    tasks_yaml = os.path.join(tasks_share, "config", "tasks.yaml")

    # 通过 xacro 展开 URDF 里 <ros2_control> 硬件标签的 $(find arm_model)；
    # 顺便把 headless 透传给 mujoco_ros2_control（纯命令行跑仿真时关掉 UI 窗口）。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            plant_urdf_path,
            " headless:=",
            LaunchConfiguration("headless"),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # O4：发布端与订阅端一起改到本工作区专用话题名，避免同 ROS domain 内别的
    # robot_state_publisher 让 controller_manager 静默加载错模型。
    robot_description_remap = [("robot_description", "arm_robot_description")]

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=robot_description_remap,
        output="screen",
    )

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            ParameterFile(controller_yaml),
            ParameterFile(mujoco_plugins_yaml),
        ],
        remappings=robot_description_remap,
        on_exit=Shutdown(),
    )

    # spawner 依赖 controller_manager 就绪，按「上一个退出后再起下一个」串成链：
    # mujoco 启动 -> joint_state_broadcaster -> (sim_pid_controller + arm_controller)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("service_call_timeout"),
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # --activate-as-group：两个控制器一次性激活，避免中间态——arm_controller 的命令
    # 接口是仿真 PID 控制器的 reference interface，需要它先就位。
    controller_chain = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--switch-timeout",
            LaunchConfiguration("switch_timeout"),
            "--service-call-timeout",
            LaunchConfiguration("service_call_timeout"),
            "--activate-as-group",
            "sim_pid_controller",
            "arm_controller",
        ],
        output="screen",
    )

    # 任务层：tasks_config 固定为安装后的 tasks.yaml，序列名由 task_sequence 指定。
    # exit_when_idle 默认 true（跑完就退出，on_exit=Shutdown 收掉整个 launch）；
    # 想常驻复用同一个进程（ros2 param set /task_runner task_sequence ...）就设 false。
    task_runner = Node(
        package="tasks",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tasks_config": LaunchConfiguration("tasks_config"),
                "task_sequence": LaunchConfiguration("task_sequence"),
                "exit_when_idle": ParameterValue(
                    LaunchConfiguration("exit_when_idle"), value_type=bool
                ),
                # 单步骤超时；tasks.yaml 里的序列都是秒级，30 s 足够，长任务再调大。
                "step_timeout": ParameterValue(
                    LaunchConfiguration("step_timeout"), value_type=float
                ),
                # ros2_controller.yaml 的 exp_state 默认是 reset：控制器激活时会先做一次
                # 启动复位（reset_duration + reset_settle 约 5 s），这段时间 current_state
                # 是 reset。tasks.yaml 的序列以 reset 开头，而 reset 只能从 idel 进入，
                # 所以必须让任务层等过启动复位。task_runner 的默认值只有 0.5 s，紧跟
                # spawner 启动会直接被拒（实测 exit 1），这里放大到 10 s。
                "controller_idle_timeout": ParameterValue(
                    LaunchConfiguration("controller_idle_timeout"), value_type=float
                ),
            }
        ],
        condition=IfCondition(LaunchConfiguration("run_task")),
        on_exit=Shutdown(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
        output="screen",
    )

    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=mujoco,
            on_start=[
                LogInfo(msg="MuJoCo ros2_control node started, spawning joint_state_broadcaster"),
                joint_state_broadcaster,
            ],
        )
    )

    load_controller_chain = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                LogInfo(msg="joint_state_broadcaster spawned, spawning arm controller chain"),
                controller_chain,
            ],
        )
    )

    run_task_after_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_chain,
            on_exit=[
                LogInfo(
                    msg=[
                        "controllers active, running task sequence '",
                        LaunchConfiguration("task_sequence"),
                        "' from tasks.yaml",
                    ],
                    condition=IfCondition(LaunchConfiguration("run_task")),
                ),
                task_runner,
            ],
        )
    )

    # 若已有一个仿真正在运行，本 launch 会再造一个 controller_manager，于是 /clock 与
    # /joint_states 各有两个发布者、时间基准互相回退，robot_state_publisher 会刷
    # "Moved backwards in time, re-publishing joint transforms!"，所以这里先给一句提示。
    double_sim_hint = LogInfo(
        msg="[提示] 本 launch 会启动 MuJoCo + 控制器 + 任务层；若已有仿真在运行，请先停掉它再启动，"
        "否则两个 /clock 与 /joint_states 发布者会造成时间戳回退刷屏。"
        "想自己控制何时跑任务时，用 run_task:=false 只起仿真，再手动 ros2 run tasks task_runner。"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "task_sequence",
                default_value="reset_then_joint",
                description="要执行的序列名（tasks.yaml 的 sequences 段）；默认 4 自由度下可用的 reset_then_joint",
            ),
            DeclareLaunchArgument(
                "tasks_config",
                default_value=tasks_yaml,
                description="任务配置文件路径，默认安装后的 tasks/config/tasks.yaml",
            ),
            DeclareLaunchArgument(
                "run_task",
                default_value="true",
                description="控制器就绪后是否自动执行任务序列；false 则只起仿真，任务手动触发",
            ),
            DeclareLaunchArgument(
                "exit_when_idle",
                default_value="true",
                description="序列跑完后 task_runner 是否退出；true 时退出会连带关掉本 launch",
            ),
            DeclareLaunchArgument(
                "step_timeout",
                default_value="30.0",
                description="任务层单步骤超时（秒）；长序列自行调大",
            ),
            DeclareLaunchArgument(
                "controller_idle_timeout",
                default_value="10.0",
                description="任务层等控制器进入 idel 的最长时间（秒）；要等过启动复位（约 5 s）",
            ),
            DeclareLaunchArgument(
                "show_rviz",
                default_value="true",
                description="是否启动 RViz2",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="是否关掉 MuJoCo 自带 UI 窗口（true = 纯命令行跑仿真）",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="各节点是否使用仿真时间",
            ),
            DeclareLaunchArgument(
                "controller_manager_timeout",
                default_value="60",
                description="等待 controller_manager 服务的秒数",
            ),
            DeclareLaunchArgument(
                "switch_timeout",
                default_value="60",
                description="等待控制器激活的秒数",
            ),
            DeclareLaunchArgument(
                "service_call_timeout",
                default_value="60",
                description="等待 controller_manager 服务响应的秒数",
            ),
            double_sim_hint,
            robot_state_pub,
            mujoco,
            load_joint_state_broadcaster,
            load_controller_chain,
            run_task_after_controllers,
            rviz2,
        ]
    )
