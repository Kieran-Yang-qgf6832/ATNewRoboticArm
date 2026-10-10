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
                              （reset -> measure -> idel），跑完即退出并关掉本 launch
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

@note 2026-10-10 重写：旧版依赖已删除的 parameter_measure 包、且没有 spawn
      joint_state_broadcaster，实际不可用。参数辨识现在由控制器 ParamterMeasureState
      + tasks 的 measure 步骤完成，不需要独立节点。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler, Shutdown
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


def generate_launch_description():
    arm_model_share = get_package_share_directory("arm_model")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_model_share, "model", "robotic_arm.urdf")
    controller_yaml = os.path.join(launch_pack_share, "config", "ros2_controller.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")
    measure_yaml = os.path.join(get_package_share_directory("tasks"), "config", "measure.yaml")

    # 默认输出：项目根目录下的 measured_for_identification.csv（不再是 /tmp）。
    default_csv_path = os.path.join(_workspace_root(), "measured_for_identification.csv")

    # 通过 xacro 处理 URDF，使 <ros2_control> 硬件标签里的 $(find arm_model) 被展开。
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # O4：发布端与订阅端一起改到本工作区专用话题名，避免同 ROS domain 内别的
    # robot_state_publisher 让 controller_manager 静默加载错模型。
    robot_description_remap = [("robot_description", "arm_robot_description")]

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="是否启动 RViz2（测量不需要界面，默认关）",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="各节点是否使用仿真时间",
    )

    csv_file_path_arg = DeclareLaunchArgument(
        "csv_file_path",
        default_value=default_csv_path,
        description="辨识测量 CSV 的输出路径，默认项目根目录",
    )

    run_measure_arg = DeclareLaunchArgument(
        "run_measure",
        default_value="true",
        description="是否在控制器就绪后自动执行 measure_only 序列；false 则只起仿真",
    )

    controller_manager_timeout_arg = DeclareLaunchArgument(
        "controller_manager_timeout",
        default_value="60",
        description="等待 controller_manager 服务的秒数",
    )

    switch_timeout_arg = DeclareLaunchArgument(
        "switch_timeout",
        default_value="60",
        description="等待控制器激活的秒数",
    )

    service_call_timeout_arg = DeclareLaunchArgument(
        "service_call_timeout",
        default_value="60",
        description="等待 controller_manager 服务响应的秒数",
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        remappings=robot_description_remap,
        output="screen",
    )

    # 关键：把 CSV 路径作为**最后一个**参数源传进去。同名参数以最后出现的为准，
    # 因此它盖过 ros2_controller.yaml 里的默认值。该字典会被 launch 写成 /** 通配的
    # params 文件，controller_manager 会连同其它 --params-file 一起转给 arm_controller。
    # 控制器在构造阶段读取该参数（ParamterMeasureState 构造函数），所以必须在这里给。
    measure_output_params = {"measure_csv_file_path": LaunchConfiguration("csv_file_path")}

    mujoco = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
            ParameterFile(controller_yaml),
            measure_output_params,
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

    return LaunchDescription(
        [
            show_rviz_arg,
            use_sim_time_arg,
            csv_file_path_arg,
            run_measure_arg,
            controller_manager_timeout_arg,
            switch_timeout_arg,
            service_call_timeout_arg,
            robot_state_pub,
            mujoco,
            load_joint_state_broadcaster,
            load_controller_chain,
            run_measure_task,
            rviz2,
        ]
    )
