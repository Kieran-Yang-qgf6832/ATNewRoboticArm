/**
 * @file task_executor.hpp
 * @brief 配置驱动的任务执行器
 *
 * 替代原先「每个任务一个 C++ 类 + TaskFSMFactory + create_task」的结构：
 * 任务定义写在 @c tasks/config/tasks.yaml 里，本类负责加载、校验与执行。
 *
 * 一条**序列**由若干**步骤**组成，步骤通过 @ref TaskContext 驱动 arm_controller
 * 切换控制模式并下发指令。新增任务 = 编辑 YAML，不需要重新编译。
 *
 * @note 本类运行在普通 ROS 节点线程，允许阻塞式服务调用与内存分配，
 *       不受 arm_controller 实时控制路径的约束。
 *
 * @see TaskContext
 * @see step_types.hpp
 */

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include "step_types.hpp"
#include "task_context.hpp"

/**
 * @brief 配置驱动的任务执行器
 *
 * 生命周期：构造 → @ref load_config → 多次 @ref run_sequence。
 * 配置在 @ref load_config 中一次性解析并**全部校验**，任何非法配置都会在
 * 机械臂运动之前被拒绝。
 */
class JointConsoleUi;

class TaskExecutor {
public:
    /**
     * @brief 构造函数
     * @param node runner 节点，用于日志与参数
     * @param ctx  任务上下文，持有与 arm_controller 的全部 ROS 接口
     * @param status_sink 任务状态出口；pos_move 的姿态漂移告警经它上报
     * @note 不持有 @p ctx 的所有权，其生命周期必须覆盖本对象
     */
    TaskExecutor(const rclcpp::Node::SharedPtr& node, TaskContext* ctx, std::function<void(const std::string&)> status_sink = {});

    /// 析构：Qt UI 窗口必须先于节点销毁
    ~TaskExecutor();

    /// @return 本次任务是否用过 Qt UI（进程退出时需要绕过 Qt/atexit 清理）
    bool used_ui() const { return used_ui_; }

    /**
     * @brief 加载并校验任务配置
     *
     * 解析 @c tool / @c points / @c positions / @c position_move / @c sequences 段，随后对每个序列的每个步骤做
     * 语义校验（目标点存在、关节数匹配、字段与类型匹配等）。任一步骤不合法即整体失败，
     * 保证不会出现「执行到一半才发现配置错误」。
     *
     * @param path  YAML 配置文件路径
     * @param error 输出：失败原因
     * @return true 加载并校验成功
     */
    bool load_config(const std::string& path, std::string* error);

    /// @return 配置中全部序列名，用于错误提示
    std::vector<std::string> sequence_names() const;

    /**
     * @brief 执行一条序列
     *
     * 失败时切回 @c idel 并返回 false，**不抛出异常、不结束进程**，
     * 调用方可以继续下发新的序列。
     *
     * @param name 序列名
     * @return true 序列全部步骤正常完成
     */
    bool run_sequence(const std::string& name);

private:
    /// 校验并解析单个步骤（把 @c to 解析成绝对关节角）
    bool validate_step(Step* step, std::string* error) const;

    /// 执行单个步骤
    bool run_step(const Step& step, std::string* error);

    /// 执行关节空间两点轨迹（joint_move）
    bool run_joint_move(const Step& step, std::string* error);

    /**
     * @brief 执行关节空间多路点轨迹（joint_waypoints）
     *
     * 切入 @c joint_traj 后把全部路点一次下发，控制器自动规划时长并回发
     * @c arm_waypoints_duration；任务层按该时长守候，结束后以最后一个路点验收。
     *
     * @param step  步骤；@c duration > 0 时作为期望总时长覆盖自动规划
     * @param error 输出：失败原因
     * @return true 轨迹走完且末态残差在容差内
     */
    bool run_joint_waypoints(const Step& step, std::string* error);

    /**
     * @brief 执行笛卡尔空间多路点轨迹（cart_waypoints）
     *
     * 路点格式为 [x, y, z, qx, qy, qz, qw]，验收只比较 xyz。
     *
     * @param step  步骤；@c duration > 0 时作为期望总时长覆盖自动规划
     * @param error 输出：失败原因
     * @return true 轨迹走完且末态位置残差在容差内
     */
    bool run_cart_waypoints(const Step& step, std::string* error);

    /**
     * @brief 执行交互式关节运动（joint_console）
     *
     * 默认在步骤内部循环读终端：**逐项**输入，一次一个数——先 6 个关节角（**度**），
     * 再 1 个执行时间（秒，回车取默认值）。校验通过后以当前实测关节角为起点下发
     * @c JointTrajCmd。输入途中回车取消本组，@c exit 结束步骤；运动过程中输入
     * @c stop 会立刻把控制器切回 @c idel（在当前位置保持）。
     *
     * 步骤 @c ui: true 时改用 Qt5 窗口：按钮把同样的输入行写进 @c ConsoleQueue，
     * 所以校验、起点、状态切换与 stop 语义与终端版完全一致；无 DISPLAY 时失败。
     *
     * @param step  步骤；@c duration 作为缺省执行时间（0 表示用节点参数）
     * @param error 输出：失败原因
     * @return true 正常结束（用户主动退出）；运动失败或控制器离开 joint_traj 时返回 false
     */
    bool run_joint_console(const Step& step, std::string* error);

    /**
     * @brief 确保控制器处于 @c joint_traj 且轨迹阶段已就绪
     *
     * 该步骤只在开头切一次状态，之后每轮发送前复用本函数：@c stop 中断与控制器异常
     * 退回 @c idel 都靠它恢复，用户不需要记额外的恢复命令。
     *
     * @param error 输出：失败原因
     * @return true 控制器已在 joint_traj
     */
    bool ensure_joint_traj(std::string* error);

    /// 执行任务空间两点轨迹（cart_move / admittance_move）
    bool run_task_space_move(const Step& step, const std::string& state_name, std::string* error);

    /**
     * @brief 执行 pos_move：位置约束的绝对目标轨迹
     *
     * 目标是基座系 [x, y, z]。可选 @c orientation 四元数 [x, y, z, w]：
     * - 缺省：下发的 6 维消息里姿态分量照抄起点，模式置为 MODE_POSITION_ONLY；
     *   结束后量测相对起点姿态的夹角，超过 @c max_drift_deg 只告警、不判失败。
     * - 给出：只取四元数的 z 轴，以起点姿态为基准构造最小旋转目标姿态，模式置为
     *   MODE_POSITION_AND_TOOL_AXIS；结束后 z 轴角误差超过全局 axis_tolerance_deg
     *   判失败（绕 z 的滚转不约束，也不再告警全姿态漂移）。
     *
     * @param precheck_rejected 输出：若控制器在**未运动**的情况下因预检拒绝而离开
     *        cart_traj，置 true；box_move 用它决定是否回退到 position-only。
     * @param strict_position 为 true 时，位置残差超过 @c step.tolerance 直接判失败；
     *        为 false 时沿用 pos_move 的“只告警”行为。
     */
    bool run_position_move(const Step& step, std::string* error, bool* precheck_rejected = nullptr,
                           bool strict_position = false);

    /**
     * @brief 执行 box_move：读取视觉目标面 TF，移动到箱体位姿
     *
     * 目标来源为 @c frame（默认 target_face）在基座系下的 TF，并同时检查
     * TargetFace 消息的 valid 与时间戳新鲜度。5 维模式（位置 + 工具 z 轴）优先：
     * link6 原点 = 面心 - tool_offset * 面 z，使工具尖端落在面心。
     * 若控制器预检在未运动时拒绝，则自动回退为 position-only（目标仍为同一
     * link6 位置，姿态不约束）；回退成功时步骤成功并上报告警。
     */
    bool run_box_move(const Step& step, std::string* error);

    /**
     * @brief 执行「切入某状态 -> 保持 duration -> 切回 idel」型步骤
     *
     * 适用于 teach（示教）与 measure（参数辨识）：控制器在该状态下自行完成全部工作，
     * 任务层只负责按时长守候，并在结束后把控制器交还给 idel。
     *
     * @param step       步骤（需含正 duration）
     * @param state_name 控制器状态名
     * @param error      输出：失败原因
     */
    bool run_timed_state(const Step& step, const std::string& state_name, std::string* error);

    /// 执行 tool 步骤：向 /tool_cmd 发布开关量
    bool run_tool(const Step& step, std::string* error);

    /// 自旋等待控制器进入指定状态；若控制器回退到 idel 则立即判失败
    bool wait_for_state(const std::string& expected, std::string* error);

    /// 自旋等待控制器回到 idel
    bool wait_for_idel(const std::string& context, std::string* error);

    /// 失败收尾：请求控制器切回 idel
    void fall_back_to_idel();

    /// 判断 tool 的 action 是否等价于「开」
    bool tool_action_is_on(const std::string& action) const;

    /// 判断 tool 的 action 是否等价于「关」
    bool tool_action_is_off(const std::string& action) const;

    rclcpp::Node::SharedPtr node_; ///< runner 节点
    TaskContext* ctx_{nullptr};    ///< 任务上下文（不持有所有权）

    /// 末端执行器开关量发布器（std_msgs/Bool，见 tool 步骤）
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr tool_pub_;

    std::map<std::string, std::vector<double>> points_;    ///< 命名关节目标点（6 维关节角）
    std::map<std::string, std::vector<double>> positions_; ///< 命名基座系绝对目标（3 维）
    std::map<std::string, std::vector<Step>> sequences_;   ///< 序列名 → 步骤列表
    std::function<void(const std::string&)> status_sink_;  ///< 任务状态出口（可为空）
    double default_max_drift_deg_{10.0};                   ///< pos_move 默认姿态漂移告警阈值（度）
    /// pos_move 工具 z 轴角容差（度）；必须与控制器 pos_move_axis_tolerance_deg 一致
    double default_axis_tolerance_deg_{1.0};
    /// box_move 的全局默认：视觉目标帧、工具尖端长度、等待与新鲜度阈值
    std::string box_frame_{"target_face"};
    double box_tool_offset_{0.039};
    double box_target_timeout_{5.0};
    double box_max_age_{0.5};
    std::string tool_name_{"tool"}; ///< 当前末端执行器名（仅日志）
    std::string tool_on_{"on"};     ///< 「开」的语义名
    std::string tool_off_{"off"};   ///< 「关」的语义名
    double step_timeout_{30.0};     ///< 单步骤超时（秒）
    double idel_hold_{0.3};         ///< 切到 idel 后的稳定时长（秒）
    double settle_duration_{0.3};   ///< 切入运动状态后的稳定时长（秒）

    /// joint_console 缺省执行时间（秒），仅在步骤未写 duration 时使用
    double console_default_duration_{3.0};
    /// joint_console 单步允许的最大关节变化量（弧度）；0 表示不限制
    double console_max_step_{0.0};
    /// joint_console 每关节下限/上限（弧度）；未配置的关节取 ±inf，即不检查
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;
    /**
     * @brief 切换状态后的稳定期（秒）
     *
     * 控制器把**实际**状态发布到 @c current_state 参数是由 500 ms wall timer 驱动的，
     * 因此 @c set_exp_state 之后立刻读该参数会读到切换前的旧值：例如请求 reset 时仍读到
     * @c idel，@ref wait_for_idel 会立刻返回假成功，步骤被跳过。
     *
     * 本值必须**大于控制器的状态刷新周期**（当前 500 ms）。若控制器侧的
     * @c state_timer_ 周期调整，这里必须同步调整，可通过 @c state_settle 参数覆盖。
     */
    double state_settle_{0.6};

    /// joint_console 的 Qt UI（仅在步骤 ui: true 时创建；销毁顺序见析构函数）
    std::unique_ptr<JointConsoleUi> console_ui_;
    bool used_ui_{false}; ///< 本次运行是否创建过 Qt UI
};
