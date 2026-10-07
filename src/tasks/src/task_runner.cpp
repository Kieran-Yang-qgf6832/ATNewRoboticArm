/**
 * @file task_runner.cpp
 * @brief 任务层状态机运行入口
 *
 * 按 @c task_sequence 参数把任务串成任务链（末尾自动补 "idel"），然后周期
 * 调度 @ref TaskFSMFactory，直到任务链走到末端或出现不可恢复错误。
 *
 * @c task_sequence 中可以显式写 "idel"（例如需要多次回到空闲位时）；而
 * teach_pendant / measure 受控制器「只能 idel 进出」限制，runner 会在它们
 * 前后自动补一个 idel，无需手写。
 *
 * 用法：
 * @code{.sh}
 * # 复位后抬升 5 cm（笛卡尔轨迹）
 * ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, cart_traj]"
 * # 复位后运动到指定关节角（关节轨迹）
 * ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, joint_traj]" \
 *   -p joint_traj_target_positions:="[0.0, 0.3, -0.3, 0.0, 0.3, 0.0]"
 * # 复位后在导纳模式下柔顺地抬升 5 cm
 * ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, admittance]"
 * # 复位后示教 15 s，再回到空闲位（前后自动补 idel）
 * ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, teach_pendant]" \
 *   -p teach_pendant_duration:=15.0
 * # 复位后做一次参数辨识（时长要覆盖控制器侧整个辨识流程）
 * ros2 run tasks task_runner --ros-args -p task_sequence:="[reset, measure]" \
 *   -p measure_duration:=20.0
 * @endcode
 *
 * 前置条件：控制器已加载并激活（例如
 * launch_pack/launch/arm_controller_test_sim.launch.py），且 joint_state_broadcaster
 * 与 robot_state_publisher 正在运行（后者提供末端位姿 TF）。
 *
 * @note 仿真时间下请附加 @c -p @c use_sim_time:=true，否则时间基准与 TF 不一致。
 * @note 进程默认**常驻**：任务失败不会结束进程；链跑完后轮询 @c task_sequence
 *       参数，改成新序列即可再执行一条链，例如：
 * @code{.sh}
 * ros2 param set /task_runner task_sequence "[joint_traj]"
 * @endcode
 *       需要"跑完就退出"的旧行为时把 @c exit_when_idle 设为 true。
 *
 * @author lyz
 * @date 2026-09-30
 * @version 0.0.0
 */

#include <algorithm>
#include <any>
#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "admittance_task.hpp"
#include "cart_traj_task.hpp"
#include "idel_task.hpp"
#include "joint_traj_task.hpp"
#include "measure_task.hpp"
#include "reset_task.hpp"
#include "teach_pendant_task.hpp"
#include "task_context.hpp"
#include "task_fsm.hpp"
#include "task_fsm_factory.hpp"

namespace {

/// 默认关节顺序，需与 ros2_controller.yaml / URDF 保持一致。
const std::vector<std::string> kDefaultJoints = {
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

/// 任务链末端任务名，runner 会自动把它接到任务链末尾。
const std::string kTerminalTaskName = "idel";

/// 已实现的任务类型，用于错误提示。
const std::vector<std::string> kSupportedTasks = {
    "idel", "reset", "cart_traj", "joint_traj", "admittance", "teach_pendant", "measure"};

/// 控制器只允许从 idel 进出这些状态，runner 会自动在任务前后各补一个 idel。
/// （ResetState / TeachPendantState / ParamterMeasureState 的 enter() 都要求
///  last_state == "idel"，exit() 也只接受 "idel"。）
const std::vector<std::string> kIdelGatedTasks = {"reset", "teach_pendant", "measure"};

/**
 * @brief 判断任务是否受「只能 idel 进出」约束
 * @param type 任务类型名
 * @return true 需要在前后补 idel 任务
 */
bool needs_idel_gate(const std::string& type) {
    return std::find(kIdelGatedTasks.begin(), kIdelGatedTasks.end(), type) != kIdelGatedTasks.end();
}

/**
 * @brief 按任务类型创建任务实例
 * @param type 任务类型名（= 任务名）
 * @param name 任务实例名
 * @param ctx  上下文，实际类型为 @c TaskContext*
 * @return 任务实例；类型不支持时返回 nullptr
 * @note 新增任务时在此加一个分支，并在 CMakeLists.txt 中加入对应 .cpp。
 */
std::unique_ptr<TaskFSM> create_task(const std::string& type, const std::string& name, const std::any& ctx) {
    if (type == "idel") {
        return std::make_unique<IdelTask>(name, ctx);
    }
    if (type == "reset") {
        return std::make_unique<ResetTask>(name, ctx);
    }
    if (type == "cart_traj") {
        return std::make_unique<CartTrajTask>(name, ctx);
    }
    if (type == "joint_traj") {
        return std::make_unique<JointTrajTask>(name, ctx);
    }
    if (type == "admittance") {
        return std::make_unique<AdmittanceTask>(name, ctx);
    }
    if (type == "teach_pendant") {
        return std::make_unique<TeachPendantTask>(name, ctx);
    }
    if (type == "measure") {
        return std::make_unique<MeasureTask>(name, ctx);
    }
    return nullptr;
}

/**
 * @brief 把字符串列表格式化为便于日志打印的形式
 * @param values 字符串列表
 * @return 形如 "reset -> cart_traj" 的字符串
 */
std::string join_names(const std::vector<std::string>& values) {
    std::string text;
    for (std::size_t i = 0; i < values.size(); ++i) {
        text += (i == 0 ? "" : " -> ");
        text += values[i];
    }
    return text;
}

}  // namespace

/**
 * @brief 任务层 runner 节点
 *
 * 参数：
 * - @c controller_node：控制器节点名（默认 "arm_controller"）；
 * - @c joints：关节顺序（默认 joint1..joint6）；
 * - @c cart_traj_topic / @c joint_traj_topic / @c admittance_topic：轨迹话题
 *   （默认 "arm_cart_traj" / "arm_joint_traj" / "arm_admittance"）；
 * - @c base_frame / @c ee_frame：末端位姿参考系与末端坐标系（默认 "base_link" / "link6"）；
 * - @c task_sequence：任务链（默认 {"reset", "cart_traj"}），可含 "idel"；
 * - @c task_status_topic：任务状态话题（默认 "task_status"），消息为
 *   @c std_msgs::msg::String，内容约定为：
 *   - @c "chain:pending <a> -> <b> -> ..."：任务链已就绪、开始执行；
 *   - @c "<task>:running"：某个任务正在执行（含链首任务）；
 *   - @c "chain:finished"：整条任务链正常走完；
 *   - @c "chain:failed: <原因>"：前置条件不满足、任务失败或整链超时；
 *   - @c "chain:idle; waiting for a new task_sequence parameter"：本链已结束，
 *     进程常驻等待新的 @c task_sequence。
 *
 *   话题为 transient_local，外部用 @c ros2 topic echo --once /task_status
 *   就能看到最近一次状态，不必抓取日志。
 * - @c step_period：调度周期（秒，默认 0.02）；
 * - @c task_timeout：整条任务链超时（秒，默认 120）；
 * - @c exit_settle：任务链结束后保持 idel 的时长（秒，默认 0.5）；
 * - @c controller_idle_timeout：切入新任务前等待控制器进入 idel 的最长时间
 *   （秒，默认 0.5）。控制器不在 idel 时任务直接报错、不发轨迹；设为 0 表示
 *   严格立即检查（代价是刚切完状态的任务可能被误判为忙）。
 * - @c startup_timeout：等待控制器与 /joint_states 就绪的超时（秒，默认 10；
 *   冷启动仿真时建议调大，launch 里默认给 60）。
 */
class TaskRunner : public rclcpp::Node {
public:
    TaskRunner()
        : rclcpp::Node("task_runner") {
        controller_node_ = declare_parameter<std::string>("controller_node", "arm_controller");
        joints_          = declare_parameter<std::vector<std::string>>("joints", kDefaultJoints);
        cart_traj_topic_  = declare_parameter<std::string>("cart_traj_topic", "arm_cart_traj");
        joint_traj_topic_ = declare_parameter<std::string>("joint_traj_topic", "arm_joint_traj");
        admittance_topic_ = declare_parameter<std::string>("admittance_topic", "arm_admittance");
        base_frame_       = declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_         = declare_parameter<std::string>("ee_frame", "link6");
        task_sequence_ =
            declare_parameter<std::vector<std::string>>("task_sequence", std::vector<std::string>{"reset", "cart_traj"});
        step_period_  = declare_parameter<double>("step_period", 0.02);
        task_timeout_ = declare_parameter<double>("task_timeout", 120.0);
        exit_settle_  = declare_parameter<double>("exit_settle", 0.5);
        startup_timeout_ = declare_parameter<double>("startup_timeout", 10.0);
        task_status_topic_ = declare_parameter<std::string>("task_status_topic", "task_status");
        controller_idle_timeout_ = declare_parameter<double>("controller_idle_timeout", 0.5);
        exit_when_idle_ = declare_parameter<bool>("exit_when_idle", false);

        // 任务状态出口：transient_local，后启动的订阅者也能立刻拿到最近一次状态。
        status_pub_ = create_publisher<std_msgs::msg::String>(task_status_topic_, rclcpp::QoS(1).transient_local());
    }

    /**
     * @brief 发布一次任务状态
     * @param text 状态文本，格式见类文档中的 @c task_status_topic 说明
     */
    void publish_status(const std::string& text) const {
        if (!status_pub_) {
            return;
        }
        std_msgs::msg::String message;
        message.data = text;
        status_pub_->publish(message);
    }

    /**
     * @brief 保持 idel 若干秒，让控制器完成状态切换
     * @param ctx      任务上下文
     * @param seconds  保持时长（秒）
     */
    void hold_idel(TaskContext& ctx, double seconds) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
        rclcpp::WallRate hold_rate(100.0);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
            ctx.spin_some();
            hold_rate.sleep();
        }
    }

    /**
     * @brief 任务链失败时的收尾：打 ERROR、发布状态、把控制器切回 idel 并保持
     * @param ctx    任务上下文
     * @param reason 失败原因
     * @note 只结束**本次**任务链，进程继续存活，可以再发下一条链。
     */
    void fail_and_fall_back_to_idel(TaskContext& ctx, const std::string& reason) {
        RCLCPP_ERROR(get_logger(), "task chain failed: %s", reason.c_str());
        publish_status("chain:failed: " + reason);
        if (!ctx.set_exp_state("idel")) {
            RCLCPP_ERROR(get_logger(), "failed to switch arm_controller back to idel");
            return;
        }
        RCLCPP_INFO(get_logger(), "arm_controller switched back to idel, holding for %.2f s", exit_settle_);
        hold_idel(ctx, exit_settle_);
    }

    /**
     * @brief 把任务序列串成一条链并设定初始任务
     * @param factory    任务工厂；任务实例只注册一次，可被后续多条链复用
     * @param task_ctx   任务构造用的上下文（实际类型为 @c TaskContext*）
     * @param sequence   任务序列，例如 {"reset", "cart_traj"}
     * @param chain_out  输出：补齐 idel 门控后的完整任务链
     * @return true 串链成功；false 任务类型未知、注册或 link 失败
     */
    bool link_chain(
        TaskFSMFactory& factory, const std::any& task_ctx, const std::vector<std::string>& sequence,
        std::vector<std::string>* chain_out) {
        // 同一类型只注册一次（任务名可重复出现在任务链中，例如链中间的 idel）。
        auto register_type = [&](const std::string& type) {
            if (factory.has_task(type)) {
                return true;
            }
            auto task = create_task(type, type, task_ctx);
            if (task == nullptr) {
                RCLCPP_ERROR(
                    get_logger(), "unknown task type %s; supported: %s", type.c_str(),
                    join_names(kSupportedTasks).c_str());
                return false;
            }
            if (!factory.register_task(std::move(task))) {
                RCLCPP_ERROR(get_logger(), "failed to register task %s", type.c_str());
                return false;
            }
            return true;
        };

        std::vector<std::string> chain;
        for (const auto& type : sequence) {
            if (!register_type(type)) {
                return false;
            }
            // teach_pendant / measure 只能 idel 进出，自动在前后各补一个 idel。
            const bool gated = needs_idel_gate(type);
            if (gated && (chain.empty() || chain.back() != kTerminalTaskName)) {
                chain.push_back(kTerminalTaskName);
            }
            chain.push_back(type);
            if (gated) {
                chain.push_back(kTerminalTaskName);
            }
        }
        if (chain.empty() || chain.back() != kTerminalTaskName) {
            chain.push_back(kTerminalTaskName);
        }
        if (!register_type(kTerminalTaskName)) {
            return false;
        }

        for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
            if (!factory.link(chain[i], chain[i + 1])) {
                RCLCPP_ERROR(get_logger(), "failed to link %s -> %s", chain[i].c_str(), chain[i + 1].c_str());
                return false;
            }
        }
        if (!factory.set_init_task(chain.front())) {
            RCLCPP_ERROR(get_logger(), "failed to set the initial task %s", chain.front().c_str());
            return false;
        }

        *chain_out = chain;
        return true;
    }

    /**
     * @brief 执行一条任务链
     * @param factory   任务工厂（复用其中的任务实例）
     * @param sequence  任务序列
     * @param ctx       任务上下文
     * @return true 任务链正常走到末端；false 任务失败或被中断
     * @note 失败时已经把控制器切回 idel，但**不结束进程**：调用方可以再发下一条链。
     */
    bool execute_chain(TaskFSMFactory& factory, const std::vector<std::string>& sequence, TaskContext& ctx) {
        std::vector<std::string> chain;
        if (!link_chain(factory, std::any(&ctx), sequence, &chain)) {
            publish_status("chain:failed: cannot build the task chain");
            return false;
        }

        RCLCPP_INFO(get_logger(), "task chain: %s", join_names(chain).c_str());
        publish_status("chain:pending " + join_names(chain));

        bool reached_terminal = false;
        std::string last_task;  ///< 上一次发布过状态的任务名，用于只在任务切换时发布
        const auto loop_start_time = std::chrono::steady_clock::now();
        rclcpp::WallRate rate(1.0 / std::max(step_period_, 1e-3));

        while (rclcpp::ok()) {
            ctx.spin_some();

            const auto now = this->now();
            if (!factory.run(now)) {
                fail_and_fall_back_to_idel(ctx, "task " + factory.current_task() + " failed to run");
                return false;
            }
            if (ctx.failed()) {
                fail_and_fall_back_to_idel(ctx, ctx.failure_message());
                return false;
            }

            // 任务切换时发布一次 "<task>:running"，配合 transient_local 即可看到当前进度。
            if (factory.current_task() != last_task) {
                last_task = factory.current_task();
                publish_status(last_task + ":running");
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "current task: %s", factory.current_task().c_str());

            if (factory.switch_count() + 1 >= chain.size()) {
                reached_terminal = true;
                break;
            }

            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start_time).count();
            if (elapsed > task_timeout_) {
                fail_and_fall_back_to_idel(
                    ctx, "task chain did not finish within " + std::to_string(static_cast<int>(task_timeout_)) + " s");
                return false;
            }

            rate.sleep();
        }

        if (!reached_terminal) {
            fail_and_fall_back_to_idel(ctx, "interrupted before reaching the terminal task");
            return false;
        }
        publish_status("chain:finished");

        // 结束时保持 idel 一小段时间，让控制器完成最后一次状态切换。
        RCLCPP_INFO(get_logger(), "task chain finished, holding idel for %.2f s", exit_settle_);
        hold_idel(ctx, exit_settle_);
        return true;
    }

    /**
     * @brief 等待下一条任务链：检测 @c task_sequence 参数是否被改成新的序列
     * @param current 当前任务序列
     * @return 新的任务序列；节点已关闭时返回空数组
     */
    std::vector<std::string> wait_for_next_sequence(const std::vector<std::string>& current) {
        rclcpp::WallRate rate(20.0);
        while (rclcpp::ok()) {
            // 等待期间也要处理参数服务回调，否则外部改不动 task_sequence。
            rclcpp::spin_some(shared_from_this());
            std::vector<std::string> sequence;
            if (get_parameter("task_sequence", sequence) && sequence != current) {
                return sequence;
            }
            rate.sleep();
        }
        return {};
    }

    /**
     * @brief 常驻执行任务链
     *
     * 启动时先执行一次 @c task_sequence；之后**进程常驻**，轮询 @c task_sequence
     * 参数，一旦被改成新的序列就再执行一条链。任务失败只结束本次链（控制器回退
     * idel、机械臂保持不动），进程不会退出，因此失败后可以直接改参数重发任务：
     * @code{.sh}
     * ros2 param set /task_runner task_sequence "[joint_traj]"
     * @endcode
     *
     * 把 @c exit_when_idle 设为 true 可恢复旧行为（链跑完立即退出）。
     *
     * @return true 正常结束（Ctrl+C）；异常由 main 捕获并转成退出码 1
     */
    bool run() {
        // 1. 构造共享上下文（整个进程复用，避免每条链重建 TF 监听）。
        TaskContext::Options options;
        options.controller_node = controller_node_;
        options.joints          = joints_;
        options.cart_traj_topic  = cart_traj_topic_;
        options.joint_traj_topic = joint_traj_topic_;
        options.admittance_topic = admittance_topic_;
        options.base_frame       = base_frame_;
        options.ee_frame         = ee_frame_;
        options.controller_idle_timeout = controller_idle_timeout_;
        auto ctx                = std::make_unique<TaskContext>(shared_from_this(), options);

        // 2. 等待控制器与关节反馈就绪；未就绪时持续重试，进程不退出。
        while (rclcpp::ok()) {
            if (ctx->wait_for_controller(5.0) && ctx->wait_for_joint_states(startup_timeout_)) {
                break;
            }
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000, "waiting for arm_controller and /joint_states before running tasks");
        }
        if (!rclcpp::ok()) {
            return true;
        }

        // 3. 任务实例只创建一次并复用；每条链只重新 link 与设定初始任务。
        TaskFSMFactory factory;
        std::vector<std::string> sequence = task_sequence_;
        while (rclcpp::ok()) {
            execute_chain(factory, sequence, *ctx);
            if (!rclcpp::ok() || exit_when_idle_) {
                break;
            }

            publish_status("chain:idle; waiting for a new task_sequence parameter");
            RCLCPP_INFO(
                get_logger(),
                "task chain done; waiting for a new task_sequence parameter (Ctrl+C to exit). "
                "e.g. ros2 param set /task_runner task_sequence \"[joint_traj]\"");
            sequence = wait_for_next_sequence(sequence);
            if (!rclcpp::ok() || sequence.empty()) {
                break;
            }
        }
        return true;
    }

private:
    std::string controller_node_;              ///< 控制器节点名
    std::vector<std::string> joints_;          ///< 关节顺序
    std::string cart_traj_topic_;              ///< 笛卡尔轨迹话题
    std::string joint_traj_topic_;             ///< 关节轨迹话题
    std::string admittance_topic_;             ///< 导纳期望轨迹话题
    std::string base_frame_;                   ///< 末端位姿参考系
    std::string ee_frame_;                     ///< 末端坐标系
    std::string task_status_topic_;            ///< 任务状态话题名
    std::vector<std::string> task_sequence_;   ///< 任务链
    double step_period_{0.02};                 ///< 调度周期（秒）
    double task_timeout_{120.0};               ///< 任务链超时（秒）
    double exit_settle_{0.5};                  ///< 结束前保持 idel 的时长（秒）
    double startup_timeout_{10.0};             ///< 等待控制器/关节反馈就绪的超时（秒）
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;  ///< 任务状态发布器（transient_local）
    double controller_idle_timeout_{0.5};            ///< 切入新任务前等待控制器进入 idel 的最长时间（秒）
    bool exit_when_idle_{false};                     ///< 任务链跑完后立即退出（默认常驻等待新链）
};

/**
 * @brief 节点入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数
 * @return 进程退出码，0 表示任务链正常完成
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node     = std::make_shared<TaskRunner>();
    int exit_code = 0;
    try {
        if (!node->run()) {
            exit_code = 1;
        }
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "task runner failed: %s", error.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
