/**
 * @file task_runner.cpp
 * @brief 任务层入口：加载配置并驱动 TaskExecutor 执行序列
 *
 * 本节点是**薄入口**：所有任务逻辑都在 @ref TaskExecutor 中，任务定义在
 * @c tasks/config/tasks.yaml 中。本文件只负责参数、上下文构造与常驻调度。
 *
 * 用法：
 * @code{.sh}
 * # 复位后回到空闲（最小可用序列）
 * ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="reset_only"
 * # 复位后运动到 ready 点
 * ros2 run tasks task_runner --ros-args -p use_sim_time:=true -p task_sequence:="reset_then_joint"
 * # 指定调试用配置
 * ros2 run tasks task_runner --ros-args -p tasks_config:=/path/to/my_tasks.yaml -p task_sequence:="pick_demo"
 * @endcode
 *
 * 进程默认**常驻**：序列失败或跑完都不会退出，改写 @c task_sequence 参数即可再执行一条：
 * @code{.sh}
 * ros2 param set /task_runner task_sequence "reset_then_cart"
 * @endcode
 * 把 @c exit_when_idle 设为 true 可改为「跑完就退出」。
 *
 * 前置条件：控制器已加载并激活（例如 launch_pack/launch/arm_controller_test_sim.launch.py），
 * 且 joint_state_broadcaster 与 robot_state_publisher 正在运行（后者提供末端位姿 TF）。
 *
 * @note 仿真时间下请附加 @c -p @c use_sim_time:=true，否则时间基准与 TF 不一致。
 *
 * @author lyz
 * @date 2026-10-07
 * @version 0.1.0
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "task_context.hpp"
#include "task_executor.hpp"

namespace {

/// 默认关节顺序，需与 ros2_controller.yaml / URDF 保持一致（当前生效模型为 4 自由度）。
const std::vector<std::string> kDefaultJoints = {"joint1", "joint2", "joint3", "joint4"};

/**
 * @brief 解析默认任务配置文件路径
 * @return 安装后的 @c share/tasks/config/tasks.yaml；包未安装时返回空串
 */
std::string default_tasks_config() {
    try {
        return ament_index_cpp::get_package_share_directory("tasks") + "/config/tasks.yaml";
    } catch (const std::exception&) {
        return {};
    }
}

/**
 * @brief 把字符串列表拼成便于日志阅读的形式
 * @param values 字符串列表
 * @return 形如 "a, b, c" 的字符串
 */
std::string join(const std::vector<std::string>& values) {
    std::string text;
    for (std::size_t i = 0; i < values.size(); ++i) {
        text += (i == 0 ? "" : ", ");
        text += values[i];
    }
    return text;
}

}  // namespace

/**
 * @brief 任务层 runner 节点
 *
 * 参数：
 * - @c tasks_config：任务配置文件路径（默认指向安装后的 @c tasks/config/tasks.yaml）；
 * - @c task_sequence：要执行的**序列名**（默认空，表示只加载配置并等待）；
 * - @c controller_node：控制器节点名（默认 "arm_controller"）；
 * - @c joints：关节顺序（默认 joint1..joint4）；
 * - @c cart_traj_topic / @c joint_traj_topic / @c admittance_topic：轨迹话题；
 * - @c base_frame / @c ee_frame：末端位姿参考系与末端坐标系（默认 "base_link" / "link4"）；
 * - @c startup_timeout：等待控制器与 /joint_states 就绪的超时（秒，默认 10）；
 * - @c exit_settle：序列结束后保持 idel 的时长（秒，默认 0.5）；
 * - @c controller_idle_timeout：切入任务前等待控制器进入 idel 的最长时间（秒，默认 0.5）；
 * - @c task_status_topic：任务状态话题（默认 "task_status"，transient_local），内容约定为：
 *   - @c "sequence:pending <name>"：开始执行某序列；
 *   - @c "sequence:finished <name>"：序列正常走完；
 *   - @c "sequence:failed: <原因>"：前置条件不满足或某步骤失败；
 *   - @c "idle; waiting for a new task_sequence parameter"：已结束，常驻等待新序列。
 */
class TaskRunner : public rclcpp::Node {
public:
    TaskRunner()
        : rclcpp::Node("task_runner") {
        controller_node_         = declare_parameter<std::string>("controller_node", "arm_controller");
        joints_                  = declare_parameter<std::vector<std::string>>("joints", kDefaultJoints);
        cart_traj_topic_         = declare_parameter<std::string>("cart_traj_topic", "arm_cart_traj");
        joint_traj_topic_        = declare_parameter<std::string>("joint_traj_topic", "arm_joint_traj");
        admittance_topic_        = declare_parameter<std::string>("admittance_topic", "arm_admittance");
        base_frame_              = declare_parameter<std::string>("base_frame", "base_link");
        ee_frame_                = declare_parameter<std::string>("ee_frame", "link4");
        tasks_config_            = declare_parameter<std::string>("tasks_config", default_tasks_config());
        task_sequence_           = declare_parameter<std::string>("task_sequence", "");
        startup_timeout_         = declare_parameter<double>("startup_timeout", 10.0);
        exit_settle_             = declare_parameter<double>("exit_settle", 0.5);
        task_status_topic_       = declare_parameter<std::string>("task_status_topic", "task_status");
        controller_idle_timeout_ = declare_parameter<double>("controller_idle_timeout", 0.5);
        exit_when_idle_          = declare_parameter<bool>("exit_when_idle", false);

        // 任务状态出口：transient_local，后启动的订阅者也能立刻拿到最近一次状态。
        status_pub_ = create_publisher<std_msgs::msg::String>(task_status_topic_, rclcpp::QoS(1).transient_local());
    }

    /**
     * @brief 发布一次任务状态
     * @param text 状态文本，格式见类文档
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
     * @param ctx     任务上下文
     * @param seconds 保持时长（秒）
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
     * @brief 等待下一条序列：检测 @c task_sequence 参数是否被改成新值
     * @param current 当前序列名
     * @return 新的序列名；节点已关闭时返回空串
     */
    std::string wait_for_next_sequence(const std::string& current) {
        rclcpp::WallRate rate(20.0);
        while (rclcpp::ok()) {
            // 等待期间也要处理参数服务回调，否则外部改不动 task_sequence。
            rclcpp::spin_some(shared_from_this());
            std::string sequence;
            if (get_parameter("task_sequence", sequence) && sequence != current) {
                return sequence;
            }
            rate.sleep();
        }
        return {};
    }

    /**
     * @brief 主流程
     * @return true 正常结束（Ctrl+C，或 @c exit_when_idle 下最后一条序列成功）；
     *         false 配置加载失败，或 @c exit_when_idle 下最后一条序列失败
     */
    bool run() {
        // 1. 构造共享上下文（整个进程复用，避免每条序列重建 TF 监听）。
        TaskContext::Options options;
        options.controller_node         = controller_node_;
        options.joints                  = joints_;
        options.cart_traj_topic         = cart_traj_topic_;
        options.joint_traj_topic        = joint_traj_topic_;
        options.admittance_topic        = admittance_topic_;
        options.base_frame              = base_frame_;
        options.ee_frame                = ee_frame_;
        options.controller_idle_timeout = controller_idle_timeout_;
        auto ctx                        = std::make_unique<TaskContext>(shared_from_this(), options);

        // 2. 加载并校验任务配置。配置错误不会自愈，直接以非零码退出，便于 launch 暴露问题。
        TaskExecutor executor(shared_from_this(), ctx.get());
        std::string config_error;
        if (!executor.load_config(tasks_config_, &config_error)) {
            RCLCPP_ERROR(get_logger(), "failed to load task config: %s", config_error.c_str());
            publish_status("sequence:failed: " + config_error);
            return false;
        }
        RCLCPP_INFO(get_logger(), "available sequences: %s", join(executor.sequence_names()).c_str());

        // 3. 等待控制器与关节反馈就绪；未就绪时持续重试，进程不退出。
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

        // 4. 常驻：执行当前 task_sequence，之后轮询参数等待新序列。
        std::string sequence = task_sequence_;
        bool last_sequence_ok = true;
        while (rclcpp::ok()) {
            if (sequence.empty()) {
                RCLCPP_INFO(get_logger(), "no task_sequence set; set it to one of: %s", join(executor.sequence_names()).c_str());
            } else {
                publish_status("sequence:pending " + sequence);
                const bool ok = executor.run_sequence(sequence);
                last_sequence_ok = ok;
                if (ok) {
                    publish_status("sequence:finished " + sequence);
                    RCLCPP_INFO(
                        get_logger(), "sequence '%s' finished, holding idel for %.2f s", sequence.c_str(), exit_settle_);
                } else {
                    // 失败时执行器已把控制器切回 idel；这里只上报并保持，进程继续存活。
                    publish_status("sequence:failed: " + sequence);
                    RCLCPP_ERROR(
                        get_logger(), "sequence '%s' failed, holding idel for %.2f s", sequence.c_str(), exit_settle_);
                }
                hold_idel(*ctx, exit_settle_);
            }

            if (exit_when_idle_) {
                break;
            }

            publish_status("idle; waiting for a new task_sequence parameter");
            RCLCPP_INFO(
                get_logger(),
                "waiting for a new task_sequence parameter (Ctrl+C to exit). "
                "e.g. ros2 param set /task_runner task_sequence \"reset_then_joint\"");
            sequence = wait_for_next_sequence(sequence);
            if (!rclcpp::ok() || sequence.empty()) {
                break;
            }
        }

        // 退出时若最后一条序列失败，用非零退出码暴露故障，避免被上层脚本当成成功。
        return last_sequence_ok;
    }

private:
    std::string controller_node_;            ///< 控制器节点名
    std::vector<std::string> joints_;        ///< 关节顺序
    std::string cart_traj_topic_;            ///< 笛卡尔轨迹话题
    std::string joint_traj_topic_;           ///< 关节轨迹话题
    std::string admittance_topic_;           ///< 导纳期望轨迹话题
    std::string base_frame_;                 ///< 末端位姿参考系
    std::string ee_frame_;                   ///< 末端坐标系
    std::string tasks_config_;               ///< 任务配置文件路径
    std::string task_sequence_;              ///< 要执行的序列名
    std::string task_status_topic_;          ///< 任务状态话题名
    double startup_timeout_{10.0};           ///< 等待控制器/关节反馈就绪的超时（秒）
    double exit_settle_{0.5};                ///< 结束前保持 idel 的时长（秒）
    double controller_idle_timeout_{0.5};    ///< 切入任务前等待 idel 的最长时间（秒）
    bool exit_when_idle_{false};             ///< 序列跑完后立即退出
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_; ///< 任务状态发布器
};

/**
 * @brief 节点入口
 * @param argc 命令行参数个数
 * @param argv 命令行参数
 * @return 进程退出码，0 表示正常结束
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
