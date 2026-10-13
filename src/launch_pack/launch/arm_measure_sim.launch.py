"""
@file arm_measure_sim.launch.py
@brief 参数辨识（measure）专用场景：一条命令完成「起仿真 → 跑测量 → 落 CSV」

启动内容（按实际启动顺序）：
  1. robot_state_publisher  ：xacro 展开 arm_model/model/robotic_arm.urdf，发布
                              /arm_robot_description 与 TF（话题名不是标准名，原因见 O4）
  2. mujoco_ros2_control    ：ros2_control_node，MuJoCo 以 SystemInterface 插件运行
  3. joint_state_broadcaster：发布 /joint_states
  4. sim_pid_controller + arm_controller：链式 PID + 状态机控制器（一次性激活）
  5. task_runner            ：执行 tasks/config/measure.yaml 的 measure_only 序列
                              （idel -> measure -> idel），跑完即退出并关掉本 launch
  6. rviz2                  ：由 show_rviz 控制，默认关（测量不需要界面）

用法：
    # 完整测量：起仿真 + 跑序列 + 落盘，结束后 launch 自动退出
    ros2 launch launch_pack arm_measure_sim.launch.py

    # 只起仿真，测量另开终端手动触发（便于反复测量、换参数）
    ros2 launch launch_pack arm_measure_sim.launch.py run_measure:=false
    ros2 run tasks task_runner --ros-args \\
        -p tasks_config:=$(ros2 pkg prefix tasks)/share/tasks/config/measure.yaml \\
        -p task_sequence:=measure_only -p use_sim_time:=true -p exit_when_idle:=true

    # 换输出位置 / 需要界面
    ros2 launch launch_pack arm_measure_sim.launch.py csv_file_path:=/tmp/foo.csv show_rviz:=true

    # 迭代辨识：用上一轮辨识出的 URDF 生成激励轨迹，测量 120 s
    ros2 launch launch_pack arm_measure_sim.launch.py \\
        controller_urdf_path:=$PWD/identified_iter_30.urdf \\
        trajectory_period:=120.0 trajectory_repeat_cnt:=1 \\
        csv_file_path:=$PWD/identify_loop2/meas_iter_01.csv step_timeout:=600.0

输出：控制器参数 measure_csv_file_path，默认 <项目根目录>/measured_for_identification.csv。
      该参数由控制器在**构造阶段**读取，所以只能用 launch 传入；只改
      ros2_controller.yaml 必须重启控制器才生效（docs/bugs.md C6）。

后续离线辨识：
    ros2 run parameter_identify identify_arm \\
        --csv <项目根目录>/measured_for_identification.csv \\
        --urdf src/arm_model/model/robotic_arm.urdf \\
        --output /tmp/robotic_arm_identified.urdf \\
        --config src/parameter_identify/config/identify.yaml \\
        --report /tmp/robotic_arm_identify_report.yaml

@note controller_urdf_path 只改**控制器内部模型**（激励轨迹生成、动力学计算），
      被控对象仍是 robot_description 里的 arm_model URDF——否则就没有"真值"可辨识了。
      注意 measure.yaml 里的 duration 必须 >= move_to_start_duration + period x repeat_cnt
      （另加激励轨迹生成时间），否则任务层会提前请求 idel。

@note 2026-10-10 重写：旧版依赖已删除的 parameter_measure 包、且没有 spawn
      joint_state_broadcaster，实际不可用。参数辨识现在由控制器 ParamterMeasureState
      + tasks 的 measure 步骤完成，不需要独立节点。

@note 2026-10-12 修：原来的做法是把 measure_* 覆盖项塞进一个 {"key": value} 字典传给
      Node(parameters=[...])。launch 会把它写成 /** 通配的 params 文件，而
      controller_manager **只**把本节点参数里 arm_controller.* 前缀的项转给控制器
      （见 controller_manager 的 load_controller），/** 下的裸参数只会落在
      controller_manager 自己身上——于是 csv_file_path / measure_trajectory_period
      这些覆盖**全部静默失效**，测量照旧用 ros2_controller.yaml 里的默认值。
      现在改成在 launch 里生成一份 node key 为 arm_controller 的补充 params 文件，
      放在 ParameterFile(controller_yaml) 之后，同名参数以后者为准。
"""

import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile


def _workspace_root():
    """定位项目根目录：同时含有 src/ 与 install/ 的那一层。

    先看本 launch 文件自身——在 --symlink-install 下它是指向 src/ 的软链，realpath
    之后直接往上找即可；再退到 launch_pack 的安装目录，最后退回当前工作目录。
    这样 CSV 一定落在项目根目录，而不取决于启动时人在哪个目录。
    """
    starts = [
        os.path.dirname(os.path.realpath(__file__)),
        os.path.dirname(os.path.realpath(get_package_share_directory("launch_pack"))),
    ]
    for start in starts:
        current = start
        for _ in range(8):
            if os.path.isdir(os.path.join(current, "src", "launch_pack")) and os.path.isdir(
                os.path.join(current, "install")
            ):
                return current
            parent = os.path.dirname(current)
            if parent == current:
                break
            current = parent
    return os.getcwd()


def _write_controller_override_file(context):
    """把 measure 相关覆盖项写成 node key 为 arm_controller 的补充 params 文件。

    key 必须是 arm_controller（而不是 launch 默认的 /**），否则 controller_manager
    不会把它转给控制器——见文件头 2026-10-12 的说明。
    """
    overrides = {
        "arm_controller": {
            "ros__parameters": {
                "urdf_path": context.perform_substitution(LaunchConfiguration("controller_urdf_path")),
                "measure_csv_file_path": context.perform_substitution(LaunchConfiguration("csv_file_path")),
                "measure_trajectory_period": float(
                    context.perform_substitution(LaunchConfiguration("trajectory_period"))
                ),
                "measure_trajectory_repeat_cnt": int(
                    context.perform_substitution(LaunchConfiguration("trajectory_repeat_cnt"))
                ),
                "measure_move_to_start_duration": float(
                    context.perform_substitution(LaunchConfiguration("move_to_start_duration"))
                ),
                # O10：0 = 每次随机（不可复现），非 0 = 固定种子。
                "measure_trajectory_seed": int(
                    context.perform_substitution(LaunchConfiguration("trajectory_seed"))
                ),
            }
        }
    }

    handle, path = tempfile.mkstemp(prefix="arm_measure_override_", suffix=".yaml")
    with os.fdopen(handle, "w", encoding="utf-8") as stream:
        yaml.safe_dump(overrides, stream, allow_unicode=True)
    return path


def _launch_setup(context):
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    plant_urdf_path = os.path.join(arm_model_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")
    measure_yaml = os.path.join(get_package_share_directory("tasks"), "config", "measure.yaml")

    controller_override_yaml = _write_controller_override_file(context)

    # 通过 xacro 处理 URDF，使 <ros2_control> 硬件标签里的 $(find arm_model) 被展开。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            plant_urdf_path,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

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

    # 参数源顺序即优先级：后面的同名参数覆盖前面的。补充文件必须放在
    # ParameterFile(controller_yaml) 之后。
    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            ParameterFile(controller_yaml),
            ParameterFile(controller_override_yaml),
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

    # 测量序列：单独一份 YAML，不影响 tasks.yaml 里的其它任务。
    # exit_when_idle 让它在序列跑完后退出，on_exit=Shutdown 随之关掉整个场景。
    task_runner = Node(
        package="tasks",
        executable="task_runner",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "tasks_config": measure_yaml,
                "task_sequence": "measure_only",
                "exit_when_idle": True,
                # measure 步骤的 duration 是任务层等待上界；测量比它长时状态机会停在
                # measure 直到跑完，因此这里的超时必须够大，否则会被误判失败。
                "step_timeout": ParameterValue(LaunchConfiguration("step_timeout"), value_type=float),
            }
        ],
        condition=IfCondition(LaunchConfiguration("run_measure")),
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

    run_measure_task = RegisterEventHandler(
        OnProcessExit(
            target_action=controller_chain,
            on_exit=[
                LogInfo(msg="controllers active, running the measure_only task sequence"),
                task_runner,
            ],
        )
    )

    return [
        robot_state_pub,
        mujoco,
        load_joint_state_broadcaster,
        load_controller_chain,
        run_measure_task,
        rviz2,
    ]


def generate_launch_description():
    default_csv_path = os.path.join(_workspace_root(), "measured_for_identification.csv")
    default_controller_urdf = os.path.join(
        get_package_share_directory("arm_model"), "model", "robotic_arm.urdf"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "show_rviz",
                default_value="false",
                description="是否启动 RViz2（测量不需要界面，默认关）",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="各节点是否使用仿真时间",
            ),
            DeclareLaunchArgument(
                "csv_file_path",
                default_value=default_csv_path,
                description="辨识测量 CSV 的输出路径，默认项目根目录",
            ),
            DeclareLaunchArgument(
                "run_measure",
                default_value="true",
                description="是否在控制器就绪后自动执行 measure_only 序列；false 则只起仿真",
            ),
            DeclareLaunchArgument(
                "controller_urdf_path",
                default_value=default_controller_urdf,
                description="控制器内部模型（激励轨迹生成用）的 URDF；被控对象仍由 robot_description 决定",
            ),
            DeclareLaunchArgument(
                "trajectory_period",
                default_value="10.0",
                description="激励轨迹周期（秒）",
            ),
            DeclareLaunchArgument(
                "trajectory_repeat_cnt",
                default_value="1",
                description="激励轨迹重复次数",
            ),
            DeclareLaunchArgument(
                "move_to_start_duration",
                default_value="3.0",
                description="从当前位形移动到激励轨迹起点的时长（秒）",
            ),
            DeclareLaunchArgument(
                "trajectory_seed",
                default_value="0",
                description="激励轨迹 CMA-ES 随机种子：0 = 随机（不可复现），非 0 = 固定可复现",
            ),
            DeclareLaunchArgument(
                "step_timeout",
                default_value="30.0",
                description="任务层单步骤超时（秒）；长测量需要放大",
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
