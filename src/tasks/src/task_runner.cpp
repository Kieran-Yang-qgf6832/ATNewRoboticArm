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
 * - @c step_period：调度周期（秒，默认 0.02）；
 * - @c task_timeout：整条任务链超时（秒，默认 120）；
 * - @c exit_settle：任务链结束后保持 idel 的时长（秒，默认 0.5）；
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
    }

    /**
     * @brief 执行整条任务链
     * @return true 任务链正常走到末端；false 前置条件不满足或任务失败
     */
    bool run() {
        // 1. 构造共享上下文并等待控制器就绪。
        TaskContext::Options options;
        options.controller_node = controller_node_;
        options.joints          = joints_;
        options.cart_traj_topic  = cart_traj_topic_;
        options.joint_traj_topic = joint_traj_topic_;
        options.admittance_topic = admittance_topic_;
        options.base_frame       = base_frame_;
        options.ee_frame         = ee_frame_;
        auto ctx                = std::make_unique<TaskContext>(shared_from_this(), options);

        // 2. 注册任务并串成任务链：sequence[0] -> ... -> idel。
        //    先校验任务链与参数，再等待控制器，参数写错时可以立即返回。
        TaskFSMFactory factory;
        const std::any task_ctx(ctx.get());

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
        for (const auto& type : task_sequence_) {
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

        RCLCPP_INFO(get_logger(), "task chain: %s", join_names(chain).c_str());

        // 3. 等待控制器与关节反馈就绪。
        if (!ctx->wait_for_controller(startup_timeout_)) {
            return false;
        }
        if (!ctx->wait_for_joint_states(startup_timeout_)) {
            return false;
        }

        // 4. 周期调度，直到走完整条任务链（切换次数达到链长 - 1）。
        bool ok               = true;
        bool reached_terminal = false;
        const auto loop_start_time = std::chrono::steady_clock::now();
        rclcpp::WallRate rate(1.0 / std::max(step_period_, 1e-3));
        while (rclcpp::ok()) {
            ctx->spin_some();

            const auto now = this->now();
            if (!factory.run(now)) {
                RCLCPP_ERROR(get_logger(), "task %s failed to run", factory.current_task().c_str());
                ok = false;
                break;
            }
            if (ctx->failed()) {
                RCLCPP_ERROR(get_logger(), "task chain aborted: %s", ctx->failure_message().c_str());
                ok = false;
                break;
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "current task: %s", factory.current_task().c_str());

            if (factory.switch_count() + 1 >= chain.size()) {
                reached_terminal = true;
                break;
            }

            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - loop_start_time).count();
            if (elapsed > task_timeout_) {
                RCLCPP_ERROR(get_logger(), "task chain did not finish within %.1f s", task_timeout_);
                ok = false;
                break;
            }

            rate.sleep();
        }

        if (!ok) {
            return false;
        }
        if (!reached_terminal) {
            RCLCPP_WARN(get_logger(), "task chain interrupted before reaching the terminal task");
            return false;
        }

        // 5. 结束时保持 idel 一小段时间，让控制器完成最后一次状态切换。
        RCLCPP_INFO(get_logger(), "task chain finished, holding idel for %.2f s", exit_settle_);
        const auto hold_deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(exit_settle_);
        rclcpp::WallRate hold_rate(100.0);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < hold_deadline) {
            ctx->spin_some();
            hold_rate.sleep();
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
    std::vector<std::string> task_sequence_;   ///< 任务链
    double step_period_{0.02};                 ///< 调度周期（秒）
    double task_timeout_{120.0};               ///< 任务链超时（秒）
    double exit_settle_{0.5};                  ///< 结束前保持 idel 的时长（秒）
    double startup_timeout_{10.0};             ///< 等待控制器/关节反馈就绪的超时（秒）
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
