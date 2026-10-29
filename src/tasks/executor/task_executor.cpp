#include "task_executor.hpp"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <deque>
#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace {

/// 控制器空闲状态名；所有任务都从这里开始、失败后回到这里。
const char kIdleState[] = "idel";

/// joint_move 默认到位容差（弧度）。
constexpr double kDefaultJointTolerance = 0.01;

/// cart_move 默认到位容差（米）。
constexpr double kDefaultCartTolerance = 0.005;

/// 把字符串列表拼成便于日志阅读的形式。
std::string join(const std::vector<std::string>& values) {
    std::string text;
    for (std::size_t i = 0; i < values.size(); ++i) {
        text += (i == 0 ? "" : ", ");
        text += values[i];
    }
    return text;
}

/// 定位用的前缀：序列名 + 步骤序号 + YAML 行号。
std::string locate(const std::string& sequence, std::size_t index, const Step& step) {
    return "sequence '" + sequence + "' step #" + std::to_string(index + 1) + " (line "
           + std::to_string(step.yaml_line) + ", type '" + step.type + "')";
}

/// 把关节角/位姿转成轨迹消息需要的 float 向量。
std::vector<float> to_float(const std::vector<double>& values) {
    std::vector<float> result;
    result.reserve(values.size());
    for (const double value : values) {
        result.push_back(static_cast<float>(value));
    }
    return result;
}

/**
 * @brief 判定一次运动是否「实际执行了」
 *
 * 残差超容差有两种截然不同的原因：
 * - 稳定后略偏离目标（例如 joint4 的已知重力静差）——属正常，只告警；
 * - 机械臂几乎没动（轨迹被控制器丢弃，例如上一次运动尚未结束）——必须判失败，
 *   否则序列会带着错误的位姿继续往下走。
 *
 * @param residual   末态残差（到目标的最大偏差）
 * @param commanded  指令位移（起点到目标的最大偏差）
 * @param moved      实际位移（起点到末态的最大偏差）
 * @param tolerance  到位容差
 * @param off_target 输出：是否为「到位但超容差」
 * @return true 看起来正常执行了
 */
bool motion_looks_executed(
    double residual, double commanded, double moved, double tolerance, bool* off_target) {
    if (residual <= tolerance) {
        *off_target = false;
        return true;
    }
    *off_target = true;
    // 指令位移本身就在容差量级时不具备判别力，按正常处理。
    if (commanded <= tolerance) {
        return true;
    }
    return moved >= 0.5 * commanded;
}

/// 边自旋边等待，保证订阅回调（/joint_states、TF）在等待期间仍被处理。
void sleep_with_spin(const rclcpp::Node::SharedPtr& node, double seconds) {
    rclcpp::WallRate rate(50.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
}

/// 圆周率常量：-std=c++17（关闭 GNU 扩展）下不保证 M_PI 可用。
constexpr double kPi = 3.14159265358979323846;

/// 角度换算系数：度 -> 弧度。
constexpr double kDegToRad = kPi / 180.0;

/**
 * @brief 定长小数格式化成文本（仅用于日志与提示）
 * @param value     数值
 * @param precision 小数位数
 * @return 形如 "12.30" 的字符串
 */
std::string format_number(double value, int precision) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

/// 把弧度格式化成一两位小数的角度文本（用于限位提示）。
std::string format_deg(double radians) {
    return format_number(radians / kDegToRad, 1);
}

/**
 * @brief 终端输入队列：全进程唯一的 stdin 读者线程
 *
 * 不能让主线程直接读 stdin：读取是阻塞的，主线程一旦阻塞就停止 spin，等待期间
 * @c /joint_states 与控制器状态都停在旧值（下一轮拿它当轨迹起点会跳变），Ctrl+C
 * 也要等回车才生效。这里用一条分离线程把整行推进队列，主线程照常自旋。
 *
 * 读者线程只访问本对象的堆上状态，不引用节点或执行器；对象故意用 @c new 创建且
 * 永不释放，使该线程在进程退出时仍阻塞在 @c getline 上也不会碰到已析构的静态对象。
 */
class ConsoleQueue {
public:
    /// @return 全进程单例
    static ConsoleQueue& instance() {
        static ConsoleQueue* queue = new ConsoleQueue();
        return *queue;
    }

    /// 把一行插回队首（运动中收到的非 stop 输入要按顺序留给下一轮）
    void push_front(const std::string& line) {
        std::lock_guard<std::mutex> lock(mutex_);
        lines_.push_front(line);
    }

    /**
     * @brief 取出一行（非阻塞）
     * @param line 输出：队首的一行（不含换行符）
     * @return true 取到一行；false 队列为空
     */
    bool try_pop(std::string* line) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (lines_.empty()) {
            return false;
        }
        *line = lines_.front();
        lines_.pop_front();
        return true;
    }

    /// @return 读者线程是否已因 EOF 结束（终端被关闭或输入被重定向到空文件）
    bool eof() const { return eof_.load(); }

private:
    ConsoleQueue() {
        std::thread([this]() {
            std::string line;
            while (std::getline(std::cin, line)) {
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    lines_.push_back(line);
                }
            }
            eof_.store(true);
        }).detach();
    }

    mutable std::mutex mutex_;  ///< 保护 lines_
    std::deque<std::string> lines_; ///< 待消费的整行
    std::atomic<bool> eof_{false};  ///< 读者线程是否已结束
};

/// 去掉首尾空白（含制表符与回车），便于解析与命令比较。
std::string trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return {};
    }
    const auto end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1);
}

/// 转小写，使 stop / STOP / Stop 等价。
std::string to_lower(std::string text) {
    for (char& ch : text) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return text;
}

/**
 * @brief 把一行文本解析成数字列表
 * @param text   已 trim 的输入行
 * @param values 输出：解析出的数字
 * @return true 全部字段都是数字（允许中间用任意空白分隔）
 */
bool parse_numbers(const std::string& text, std::vector<double>* values) {
    values->clear();
    std::istringstream stream(text);
    double value = 0.0;
    while (stream >> value) {
        values->push_back(value);
    }
    // 出现非数字字段时流会停在错误位，eof 不会置位。
    return !values->empty() && stream.eof();
}

}  // namespace

TaskExecutor::TaskExecutor(const rclcpp::Node::SharedPtr& node, TaskContext* ctx)
    : node_(node)
    , ctx_(ctx) {
    if (node_ != nullptr) {
        step_timeout_    = node_->declare_parameter<double>("step_timeout", step_timeout_);
        idel_hold_       = node_->declare_parameter<double>("idel_hold", idel_hold_);
        state_settle_    = node_->declare_parameter<double>("state_settle", state_settle_);
        settle_duration_ = node_->declare_parameter<double>("settle_duration", settle_duration_);
        // joint_console 的兵底时长：步骤没写 duration 时才生效。
        console_default_duration_ = node_->declare_parameter<double>("console_default_duration", console_default_duration_);

        // 关节限位默认全开（±inf = 不检查），由 tasks.yaml 的 joint_limits 段收紧。
        const std::size_t joint_count = ctx_ != nullptr ? ctx_->options().joints.size() : 0;
        joint_min_.assign(joint_count, -std::numeric_limits<double>::infinity());
        joint_max_.assign(joint_count, std::numeric_limits<double>::infinity());

        // 末端执行器开关量出口。话题名与缺陷文档中约定的 /tool_cmd 一致；
        // 用 transient_local 让晚启动的订阅者（例如实机侧硬件接口）也能拿到最近一次状态。
        const std::string tool_topic = node_->declare_parameter<std::string>("tool_cmd_topic", "tool_cmd");
        tool_pub_ = node_->create_publisher<std_msgs::msg::Bool>(tool_topic, rclcpp::QoS(10).transient_local());
    }
}

std::vector<std::string> TaskExecutor::sequence_names() const {
    std::vector<std::string> names;
    names.reserve(sequences_.size());
    for (const auto& entry : sequences_) {
        names.push_back(entry.first);
    }
    return names;
}

bool TaskExecutor::load_config(const std::string& path, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (ctx_ == nullptr) {
        return fail("internal error: TaskExecutor requires a TaskContext");
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& exception) {
        return fail("cannot load task config '" + path + "': " + exception.what());
    }

    points_.clear();
    sequences_.clear();
    tool_name_ = "tool";
    tool_on_   = "on";
    tool_off_  = "off";

    try {
        // ---- tool：当前末端执行器的语义名（只影响 tool 步骤的 action 与日志）----
        if (root["tool"]) {
            const YAML::Node& tool = root["tool"];
            if (!tool.IsMap()) {
                return fail("'tool' must be a mapping");
            }
            if (tool["name"]) {
                tool_name_ = tool["name"].as<std::string>();
            }
            if (tool["on"]) {
                tool_on_ = tool["on"].as<std::string>();
            }
            if (tool["off"]) {
                tool_off_ = tool["off"].as<std::string>();
            }
        }

        // ---- points：命名目标点 ----
        if (root["points"]) {
            const YAML::Node& points = root["points"];
            if (!points.IsMap()) {
                return fail("'points' must be a mapping of name -> joint values");
            }
            for (const auto& entry : points) {
                const std::string name = entry.first.as<std::string>();
                if (!entry.second.IsSequence()) {
                    return fail("point '" + name + "' must be a list of joint values");
                }
                points_[name] = entry.second.as<std::vector<double>>();
            }
        }

        // ---- joint_limits：joint_console 的输入门禁（整段可省略 = 不检查）----
        //
        // 控制器 joint_traj 只校验点数与维度，不查关节限位：手滑把 30 输成 300 会真的
        // 整条下发。这里在任务层先挡一道，超限只提示重输，不影响序列状态。
        const std::size_t joint_count = ctx_->options().joints.size();
        console_max_step_ = 0.0;
        joint_min_.assign(joint_count, -std::numeric_limits<double>::infinity());
        joint_max_.assign(joint_count, std::numeric_limits<double>::infinity());

        if (root["joint_limits"]) {
            const YAML::Node& limits = root["joint_limits"];
            if (!limits.IsMap()) {
                return fail("'joint_limits' must be a mapping");
            }
            for (const auto& entry : limits) {
                const std::string key = entry.first.as<std::string>();
                if (key == "max_step_deg") {
                    if (!entry.second.IsScalar()) {
                        return fail("'joint_limits.max_step_deg' must be a number");
                    }
                    const double max_step_deg = entry.second.as<double>();
                    if (max_step_deg < 0.0) {
                        return fail("'joint_limits.max_step_deg' must be non-negative");
                    }
                    console_max_step_ = max_step_deg * kDegToRad;
                    continue;
                }
                if (key != "joints") {
                    return fail("unknown field '" + key + "' in 'joint_limits' (expected 'max_step_deg' or 'joints')");
                }
                if (!entry.second.IsMap()) {
                    return fail("'joint_limits.joints' must be a mapping of joint name -> {min, max}");
                }
                for (const auto& joint_entry : entry.second) {
                    const std::string joint_name = joint_entry.first.as<std::string>();
                    const auto it = std::find(ctx_->options().joints.begin(), ctx_->options().joints.end(), joint_name);
                    if (it == ctx_->options().joints.end()) {
                        return fail(
                            "'joint_limits' refers to unknown joint '" + joint_name + "'; known joints: "
                            + join(ctx_->options().joints));
                    }
                    const auto index = static_cast<std::size_t>(std::distance(ctx_->options().joints.begin(), it));
                    const YAML::Node& range = joint_entry.second;
                    if (!range.IsMap() || !range["min"] || !range["max"]) {
                        return fail("joint limit '" + joint_name + "' must be a mapping with 'min' and 'max'");
                    }
                    const double min_deg = range["min"].as<double>();
                    const double max_deg = range["max"].as<double>();
                    if (!(min_deg < max_deg)) {
                        return fail("joint limit '" + joint_name + "' requires min < max");
                    }
                    joint_min_[index] = min_deg * kDegToRad;
                    joint_max_[index] = max_deg * kDegToRad;
                }
            }
        }

        // ---- sequences：序列 -> 步骤列表 ----
        if (!root["sequences"] || !root["sequences"].IsMap()) {
            return fail("task config must contain a 'sequences' mapping");
        }

        for (const auto& entry : root["sequences"]) {
            const std::string sequence_name = entry.first.as<std::string>();
            const YAML::Node& sequence      = entry.second;

            if (!sequence.IsMap() || !sequence["steps"]) {
                return fail("sequence '" + sequence_name + "' must be a mapping containing 'steps'");
            }
            const YAML::Node& steps = sequence["steps"];
            if (!steps.IsSequence() || steps.size() == 0) {
                return fail("sequence '" + sequence_name + "' must have a non-empty 'steps' list");
            }

            std::vector<Step> parsed;
            parsed.reserve(steps.size());
            for (std::size_t i = 0; i < steps.size(); ++i) {
                Step step;
                std::string step_error;
                if (!parse_step(steps[i], &step, &step_error)) {
                    return fail("sequence '" + sequence_name + "' step #" + std::to_string(i + 1) + ": " + step_error);
                }
                // 语义校验在加载阶段一次做完，保证机械臂不会执行到一半才发现配置错误。
                if (!validate_step(&step, &step_error)) {
                    return fail(locate(sequence_name, i, step) + ": " + step_error);
                }
                parsed.push_back(std::move(step));
            }
            sequences_[sequence_name] = std::move(parsed);
        }
    } catch (const std::exception& exception) {
        return fail(std::string("malformed task config: ") + exception.what());
    }

    RCLCPP_INFO(
        node_->get_logger(), "loaded task config '%s': %zu point(s), %zu sequence(s): %s", path.c_str(), points_.size(),
        sequences_.size(), join(sequence_names()).c_str());
    return true;
}

bool TaskExecutor::validate_step(Step* step, std::string* error) const {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (step == nullptr) {
        return fail("internal error: null step");
    }

    const std::size_t joint_count = ctx_->options().joints.size();

    // 字段与类型必须匹配：写错字段名比写错值更难发现，这里直接拒绝。
    const bool uses_target = !step->target.empty();
    const bool uses_to     = !step->to.empty();
    const bool uses_offset = !step->offset.empty();
    const bool uses_action = !step->action.empty();

    if (step->type == "joint_move") {
        if (uses_offset || uses_action) {
            return fail("joint_move accepts only 'to' or 'target'");
        }
        if (uses_to) {
            const auto point = points_.find(step->to);
            if (point == points_.end()) {
                std::vector<std::string> names;
                names.reserve(points_.size());
                for (const auto& entry : points_) {
                    names.push_back(entry.first);
                }
                return fail("references unknown point '" + step->to + "'; known points: " + join(names));
            }
            step->target = point->second;
        }
        if (!uses_to && !uses_target) {
            return fail("joint_move requires either 'to' (a point name) or 'target' (joint values)");
        }
        if (step->target.size() != joint_count) {
            return fail(
                "joint_move needs " + std::to_string(joint_count) + " joint values, got "
                + std::to_string(step->target.size()));
        }
        for (const double value : step->target) {
            if (!std::isfinite(value)) {
                return fail("joint_move target contains a non-finite value");
            }
        }
        if (!(step->tolerance > 0.0)) {
            step->tolerance = kDefaultJointTolerance;
        }
        return true;
    }

    if (step->type == "cart_move" || step->type == "admittance_move") {
        if (uses_to || uses_target || uses_action) {
            return fail(step->type + " accepts only 'offset'");
        }
        if (!uses_offset) {
            return fail(step->type + " requires an 'offset' (relative end-effector displacement)");
        }
        if (step->offset.size() != 3) {
            return fail(
                step->type + " needs a 3-element 'offset', got " + std::to_string(step->offset.size()));
        }
        for (const double value : step->offset) {
            if (!std::isfinite(value)) {
                return fail(step->type + " offset contains a non-finite value");
            }
        }
        if (!(step->tolerance > 0.0)) {
            step->tolerance = kDefaultCartTolerance;
        }
        return true;
    }

    if (step->type == "tool") {
        if (uses_to || uses_target || uses_offset) {
            return fail("tool accepts only 'action'");
        }
        if (!uses_action) {
            return fail("tool requires an 'action'");
        }
        if (!tool_action_is_on(step->action) && !tool_action_is_off(step->action)) {
            return fail(
                "tool action '" + step->action + "' is neither '" + tool_on_ + "'/'on' nor '" + tool_off_
                + "'/'off'");
        }
        return true;
    }

    if (step->type == "joint_console") {
        if (uses_to || uses_target || uses_offset || uses_action) {
            return fail("joint_console accepts only 'duration' (the default motion time) and 'tolerance'");
        }
        // duration 是缺省执行时间，可省略；写了就必须为正，0 表示「用节点参数」。
        if (step->duration < 0.0) {
            return fail("joint_console 'duration' must be non-negative");
        }
        if (!(step->tolerance > 0.0)) {
            step->tolerance = kDefaultJointTolerance;
        }
        return true;
    }

    // wait / idel / reset / teach / measure 不接受任何额外字段。
    if (uses_to || uses_target || uses_offset || uses_action) {
        return fail(step->type + " accepts no fields other than 'duration' and 'tolerance'");
    }
    return true;
}

bool TaskExecutor::tool_action_is_on(const std::string& action) const {
    return action == tool_on_ || action == "on";
}

bool TaskExecutor::tool_action_is_off(const std::string& action) const {
    return action == tool_off_ || action == "off";
}

bool TaskExecutor::wait_for_idel(const std::string& context, std::string* error) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(step_timeout_);
    rclcpp::WallRate rate(20.0);
    std::string state;

    while (rclcpp::ok()) {
        ctx_->spin_some();
        state = ctx_->controller_state(0.5);
        if (state == kIdleState) {
            return true;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            if (error != nullptr) {
                *error = context + " did not reach '" + kIdleState + "' within " + std::to_string(step_timeout_)
                         + " s (current state: " + (state.empty() ? "<unknown>" : state) + ")";
            }
            return false;
        }
        rate.sleep();
    }

    if (error != nullptr) {
        *error = context + " was interrupted";
    }
    return false;
}

void TaskExecutor::fall_back_to_idel() {
    if (ctx_->set_exp_state(kIdleState)) {
        RCLCPP_INFO(node_->get_logger(), "arm_controller switched back to idel after the failure");
    } else {
        RCLCPP_ERROR(node_->get_logger(), "failed to switch arm_controller back to idel");
    }
}

bool TaskExecutor::wait_for_state(const std::string& expected, std::string* error) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(step_timeout_);
    rclcpp::WallRate rate(20.0);
    std::string state;

    while (rclcpp::ok()) {
        ctx_->spin_some();
        state = ctx_->controller_state(0.5);
        if (state == expected) {
            return true;
        }
        // 目标不是 idel 却读到 idel：控制器拒绝了这次切入，或已因故障回退，无需等满超时。
        if (state == kIdleState) {
            if (error != nullptr) {
                *error = "arm_controller stayed in idel instead of entering '" + expected + "'";
            }
            return false;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
            if (error != nullptr) {
                *error = "arm_controller did not enter '" + expected + "' within " + std::to_string(step_timeout_)
                         + " s (current state: " + (state.empty() ? "<unknown>" : state) + ")";
            }
            return false;
        }
        rate.sleep();
    }

    if (error != nullptr) {
        *error = "interrupted while waiting for state '" + expected + "'";
    }
    return false;
}

bool TaskExecutor::run_joint_move(const Step& step, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (!ctx_->set_exp_state("joint_traj")) {
        return fail("failed to switch arm_controller to joint_traj");
    }
    sleep_with_spin(node_, state_settle_);
    if (!wait_for_state("joint_traj", error)) {
        return false;
    }
    sleep_with_spin(node_, settle_duration_);

    if (!ctx_->have_joint_states()) {
        return fail("no valid /joint_states before publishing the joint trajectory");
    }
    // 起点必须取当前实测关节角：控制器以当前位形为逆解种子，首点与实测不一致会直接跳变。
    const std::vector<double> start = ctx_->joint_positions();
    if (start.size() != step.target.size()) {
        return fail("joint state size does not match the target size");
    }

    const std::vector<std::vector<float>> points{to_float(start), to_float(step.target)};
    const std::vector<float> seconds{0.0f, static_cast<float>(step.duration)};
    if (!ctx_->publish_joint_traj(points, seconds)) {
        return fail("failed to publish the joint trajectory");
    }
    RCLCPP_INFO(node_->get_logger(), "joint trajectory published, moving in %.2f s", step.duration);

    sleep_with_spin(node_, step.duration);

    // 控制器若回退到 idel，说明这次运动没有走完，必须上报。
    const std::string state = ctx_->controller_state(2.0);
    if (state != "joint_traj") {
        return fail(
            "arm_controller left joint_traj (now '" + (state.empty() ? std::string("<unknown>") : state)
            + "'); see the controller log for the failing stage (inverse kinematics / inverse dynamics)");
    }

    if (ctx_->have_joint_states() && ctx_->joint_positions().size() == step.target.size()) {
        const std::vector<double>& settled = ctx_->joint_positions();
        double residual = 0.0;
        double commanded = 0.0;
        double moved = 0.0;
        for (std::size_t i = 0; i < settled.size(); ++i) {
            residual  = std::max(residual, std::abs(settled[i] - step.target[i]));
            commanded = std::max(commanded, std::abs(step.target[i] - start[i]));
            moved     = std::max(moved, std::abs(settled[i] - start[i]));
        }

        bool off_target = false;
        if (!motion_looks_executed(residual, commanded, moved, step.tolerance, &off_target)) {
            return fail(
                "joint trajectory appears to have been ignored: moved " + std::to_string(moved)
                + " rad of the commanded " + std::to_string(commanded) + " rad");
        }
        if (off_target) {
            RCLCPP_WARN(
                node_->get_logger(), "joint trajectory finished with a residual joint error of %.4f rad (tolerance %.4f rad)",
                residual, step.tolerance);
        } else {
            RCLCPP_INFO(node_->get_logger(), "joint trajectory finished, max joint error %.4f rad", residual);
        }
    }
    return true;
}

bool TaskExecutor::ensure_joint_traj(std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (ctx_->controller_state(2.0) == "joint_traj") {
        return true;
    }

    // 不在 joint_traj 就往那儿切：stop 中断后控制器回到 idel，控制器异常退出也走这条路。
    if (!ctx_->set_exp_state("joint_traj")) {
        return fail("failed to switch arm_controller to joint_traj");
    }
    sleep_with_spin(node_, state_settle_);
    if (!wait_for_state("joint_traj", error)) {
        return false;
    }
    sleep_with_spin(node_, settle_duration_);
    return true;
}

bool TaskExecutor::run_joint_console(const Step& step, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (ctx_ == nullptr) {
        return fail("internal error: joint_console requires a TaskContext");
    }

    const std::vector<std::string>& joints = ctx_->options().joints;
    const std::size_t joint_count          = joints.size();
    const double default_duration          = step.duration > 0.0 ? step.duration : console_default_duration_;

    if (!ensure_joint_traj(error)) {
        return false;
    }

    // 关节顺序与限位先打一遍：用户不必凭记忆确认哪一列对应哪个关节。
    std::string table;
    for (std::size_t i = 0; i < joint_count; ++i) {
        if (i != 0) {
            table += ", ";
        }
        table += joints[i];
        if (std::isfinite(joint_min_[i]) && std::isfinite(joint_max_[i])) {
            table += "[" + format_deg(joint_min_[i]) + ", " + format_deg(joint_max_[i]) + "]";
        }
    }
    RCLCPP_INFO(
        node_->get_logger(),
        "joint_console: 逐项输入 %zu 个关节角（度），再输入执行时间（秒，回车取默认 %.2f）；"
        "输入途中回车取消本组，exit 结束，运动中 stop 立即停止",
        joint_count, default_duration);
    RCLCPP_INFO(node_->get_logger(), "joint_console: 关节顺序与限位（度） %s", table.c_str());
    if (console_max_step_ > 0.0) {
        RCLCPP_INFO(node_->get_logger(), "joint_console: 单步最大变化 %s 度", format_deg(console_max_step_).c_str());
    }

    // 运动期间收到、但本轮不消费的输入；按原顺序留给下一轮，避免丢指令。
    std::deque<std::string> pending;
    rclcpp::WallRate idle_rate(50.0);

    // 取一行：优先进运动期间排队的输入，其次等待新行；等待期间照常自旋。
    // 返回 false 表示终端已关闭或进程正在退出。
    const auto next_line = [&](std::string* line) {
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
            if (!pending.empty()) {
                *line = pending.front();
                pending.pop_front();
                return true;
            }
            if (ConsoleQueue::instance().try_pop(line)) {
                return true;
            }
            if (ConsoleQueue::instance().eof()) {
                return false;
            }
            idle_rate.sleep();
        }
        return false;
    };

    while (rclcpp::ok()) {
        RCLCPP_INFO(
            node_->get_logger(), "joint_console> 开始输入：逐项敲 %zu 个关节角（度），再敲执行时间（秒）", joint_count);

        // 1. 逐个关节输入角度：一次只收一个数，避免行内看错第几列，也能逐项校验限位。
        std::vector<double> target(joint_count, 0.0);
        bool cancelled = false;
        bool quit      = false;
        for (std::size_t i = 0; i < joint_count && !cancelled && !quit;) {
            RCLCPP_INFO(
                node_->get_logger(), "  [%zu/%zu] %s 角度 (deg)，回车取消本组: ", i + 1, joint_count, joints[i].c_str());
            std::string line;
            if (!next_line(&line)) {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 终端输入已关闭，结束步骤");
                return true;
            }

            const std::string command = to_lower(trim(line));
            if (command == "exit" || command == "quit") {
                quit = true;
                break;
            }
            if (command == "stop") {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 当前没有运动，无需停止");
                continue; // 重新提示同一个关节
            }
            if (command.empty()) {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 已取消本组输入");
                cancelled = true;
                break;
            }

            std::vector<double> values;
            if (!parse_numbers(command, &values)) {
                RCLCPP_WARN(node_->get_logger(), "joint_console: 无法解析 '%s'，这里只收一个数字", command.c_str());
                continue;
            }
            if (values.size() != 1) {
                RCLCPP_WARN(
                    node_->get_logger(), "joint_console: 一次只输入一个数，收到 %zu 个；请重输 %s", values.size(),
                    joints[i].c_str());
                continue;
            }
            if (!std::isfinite(values.front())) {
                RCLCPP_WARN(node_->get_logger(), "joint_console: %s 的角度不是有限值", joints[i].c_str());
                continue;
            }

            const double radians = values.front() * kDegToRad;
            if (radians < joint_min_[i] || radians > joint_max_[i]) {
                RCLCPP_WARN(
                    node_->get_logger(), "joint_console: %s = %s deg 超出限位 [%s, %s] deg，请重输", joints[i].c_str(),
                    format_number(values.front(), 1).c_str(), format_deg(joint_min_[i]).c_str(),
                    format_deg(joint_max_[i]).c_str());
                continue;
            }

            target[i] = radians;
            RCLCPP_INFO(
                node_->get_logger(), "  %s = %s deg (%s rad)", joints[i].c_str(), format_number(values.front(), 1).c_str(),
                format_number(radians, 4).c_str());
            ++i;
        }
        if (quit) {
            RCLCPP_INFO(node_->get_logger(), "joint_console: 结束交互");
            return true;
        }
        if (cancelled) {
            continue;
        }

        // 2. 执行时间：同样一次一个数，直接回车取默认值。
        double duration  = default_duration;
        bool duration_ok = false;
        while (!duration_ok && rclcpp::ok()) {
            RCLCPP_INFO(
                node_->get_logger(), "  [%zu/%zu] 执行时间 (s)，回车取默认 %s: ", joint_count + 1, joint_count + 1,
                format_number(default_duration, 2).c_str());
            std::string line;
            if (!next_line(&line)) {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 终端输入已关闭，结束步骤");
                return true;
            }

            const std::string command = to_lower(trim(line));
            if (command == "exit" || command == "quit") {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 结束交互");
                return true;
            }
            if (command == "stop") {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 当前没有运动，无需停止");
                continue;
            }
            if (command.empty()) {
                RCLCPP_INFO(
                    node_->get_logger(), "joint_console: 执行时间用默认值 %s s", format_number(duration, 2).c_str());
                duration_ok = true;
                break;
            }

            std::vector<double> values;
            if (!parse_numbers(command, &values)) {
                RCLCPP_WARN(node_->get_logger(), "joint_console: 无法解析 '%s'，这里只收一个数字", command.c_str());
                continue;
            }
            if (values.size() != 1) {
                RCLCPP_WARN(node_->get_logger(), "joint_console: 一次只输入一个数，收到 %zu 个", values.size());
                continue;
            }
            if (!(values.front() > 0.0) || !std::isfinite(values.front())) {
                RCLCPP_WARN(
                    node_->get_logger(), "joint_console: 执行时间必须为正数，收到 %s",
                    format_number(values.front(), 3).c_str());
                continue;
            }
            duration    = values.front();
            duration_ok = true;
        }
        if (!duration_ok) {
            return fail("interrupted while waiting for terminal input");
        }

        // 3. 单步幅度相对**最新实测角**判断：这是拦住手滑（例如把 30 输成 300）的第二道闸。
        if (!ctx_->have_joint_states() || ctx_->joint_positions().size() != joint_count) {
            return fail("no valid /joint_states before publishing the joint trajectory");
        }
        if (console_max_step_ > 0.0) {
            const std::vector<double>& before = ctx_->joint_positions();
            bool too_far                      = false;
            for (std::size_t i = 0; i < joint_count; ++i) {
                const double delta = std::abs(target[i] - before[i]);
                if (delta > console_max_step_) {
                    RCLCPP_WARN(
                        node_->get_logger(), "joint_console: %s 单步变化 %s deg，超过上限 %s deg；本次输入作废",
                        joints[i].c_str(), format_number(delta / kDegToRad, 1).c_str(),
                        format_deg(console_max_step_).c_str());
                    too_far = true;
                    break;
                }
            }
            if (too_far) {
                continue;
            }
        }

        // 4. 回显整组目标（度 + 弧度）与执行时间，避免单位误读。
        std::string echo;
        for (std::size_t i = 0; i < joint_count; ++i) {
            echo += (i == 0 ? "" : ", ");
            echo += joints[i] + "=" + format_number(target[i] / kDegToRad, 1) + "deg/" + format_number(target[i], 4)
                    + "rad";
        }
        RCLCPP_INFO(node_->get_logger(), "joint_console: 目标 %s", echo.c_str());
        RCLCPP_INFO(node_->get_logger(), "joint_console: 执行时间 %.2f s", duration);

        // 5. 发送前确认状态：stop 之后或控制器异常退回 idel 时自动重新切入。
        if (!ensure_joint_traj(error)) {
            return false;
        }
        if (!ctx_->have_joint_states() || ctx_->joint_positions().size() != joint_count) {
            return fail("no valid /joint_states before publishing the joint trajectory");
        }
        // 起点取当前实测角：与实测不一致时控制器会在 t=0 跳变。
        const std::vector<double> start = ctx_->joint_positions();

        const std::vector<std::vector<float>> points{to_float(start), to_float(target)};
        const std::vector<float> seconds{0.0f, static_cast<float>(duration)};
        if (!ctx_->publish_joint_traj(points, seconds)) {
            return fail("failed to publish the joint trajectory");
        }
        RCLCPP_INFO(node_->get_logger(), "joint_console: 轨迹已下发，运动中可输入 stop");

        // 6. 运动中等待：只认 stop（切 idel 就地保持），其余输入排队给下一轮。
        bool stopped = false;
        {
            rclcpp::WallRate rate(50.0);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration);
            while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
                rclcpp::spin_some(node_);
                std::string queued;
                while (ConsoleQueue::instance().try_pop(&queued)) {
                    if (to_lower(trim(queued)) == "stop") {
                        if (ctx_->set_exp_state(kIdleState)) {
                            RCLCPP_WARN(node_->get_logger(), "joint_console: 收到 stop，控制器将停在当前位姿并切回 idel");
                            // 必须等它**真的**到 idel 再继续：current_state 由控制器的
                            // 500 ms timer 刷新，立刻读会读回 joint_traj，下一轮就会把轨迹
                            // 发给已经退出的状态机实例，那条指令会被静默丢弃。
                            sleep_with_spin(node_, state_settle_);
                            wait_for_idel("stop", nullptr);
                        } else {
                            RCLCPP_ERROR(node_->get_logger(), "joint_console: 收到 stop，但切换 idel 失败");
                        }
                        stopped = true;
                        break;
                    }
                    pending.push_back(queued);
                }
                if (stopped) {
                    break;
                }
                rate.sleep();
            }
        }
        if (!rclcpp::ok()) {
            return fail("interrupted while moving");
        }
        if (stopped) {
            // 下一轮的 ensure_joint_traj 会自动把控制器切回 joint_traj。
            continue;
        }

        // 7. 结束后核对：控制器若已回退到 idel，说明这次运动没有走完。
        const std::string state = ctx_->controller_state(2.0);
        if (state != "joint_traj") {
            return fail(
                "arm_controller left joint_traj (now '" + (state.empty() ? std::string("<unknown>") : state)
                + "'); see the controller log for the failing stage (inverse kinematics / inverse dynamics)");
        }

        // 到位判定口径与 joint_move 一致：被忽略 -> 失败；到位但超容差 -> 告警。
        if (ctx_->have_joint_states() && ctx_->joint_positions().size() == joint_count) {
            const std::vector<double>& settled = ctx_->joint_positions();
            double residual                    = 0.0;
            double commanded                   = 0.0;
            double moved                       = 0.0;
            for (std::size_t i = 0; i < joint_count; ++i) {
                residual  = std::max(residual, std::abs(settled[i] - target[i]));
                commanded = std::max(commanded, std::abs(target[i] - start[i]));
                moved     = std::max(moved, std::abs(settled[i] - start[i]));
            }

            bool off_target = false;
            if (!motion_looks_executed(residual, commanded, moved, step.tolerance, &off_target)) {
                return fail(
                    "joint trajectory appears to have been ignored: moved " + std::to_string(moved)
                    + " rad of the commanded " + std::to_string(commanded) + " rad");
            }
            if (off_target) {
                RCLCPP_WARN(
                    node_->get_logger(), "joint_console: 到位残差 %.4f rad（容差 %.4f rad）", residual, step.tolerance);
            } else {
                RCLCPP_INFO(node_->get_logger(), "joint_console: 到位，最大关节误差 %.4f rad", residual);
            }
        }
    }

    return true;
}

bool TaskExecutor::run_task_space_move(const Step& step, const std::string& state_name, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (!ctx_->set_exp_state(state_name)) {
        return fail("failed to switch arm_controller to " + state_name);
    }
    sleep_with_spin(node_, state_settle_);
    if (!wait_for_state(state_name, error)) {
        return false;
    }
    sleep_with_spin(node_, settle_duration_);

    // 起点必须取当前实测末端位姿：轨迹首点若与当前位姿不一致，机械臂会在 t=0 直接跳过去。
    std::vector<double> start_pose;
    if (!ctx_->end_effector_pose(&start_pose)) {
        return fail("cannot read the end-effector pose before publishing the task space trajectory (TF unavailable)");
    }

    std::vector<double> target_pose = start_pose;
    for (std::size_t i = 0; i < step.offset.size() && i < target_pose.size(); ++i) {
        target_pose[i] += step.offset[i];
    }

    const std::vector<std::vector<float>> points{to_float(start_pose), to_float(target_pose)};
    const std::vector<float> seconds{0.0f, static_cast<float>(step.duration)};
    const bool published =
        (state_name == "admittance") ? ctx_->publish_admittance(points, seconds) : ctx_->publish_cart_traj(points, seconds);
    if (!published) {
        return fail("failed to publish the task space trajectory");
    }
    RCLCPP_INFO(
        node_->get_logger(), "%s: start [%.3f %.3f %.3f], target [%.3f %.3f %.3f], moving in %.2f s", state_name.c_str(),
        start_pose[0], start_pose[1], start_pose[2], target_pose[0], target_pose[1], target_pose[2], step.duration);

    sleep_with_spin(node_, step.duration);

    // 控制器若回退到 idel，说明这次运动没有走完，必须上报。具体原因（目标不可达 /
    // 逆解数值失败）由控制器在日志里给出：笛卡尔轨迹在**下发前**就会做可达性预检，
    // 目标不可达时一个运动指令都不会下发（docs/bugs.md O2）。
    const std::string state = ctx_->controller_state(2.0);
    if (state != state_name) {
        return fail(
            "arm_controller left " + state_name + " (now '" + (state.empty() ? std::string("<unknown>") : state)
            + "'); check the controller log — an unreachable Cartesian target is rejected before any motion");
    }

    std::vector<double> current_pose;
    if (ctx_->end_effector_pose(&current_pose)) {
        double residual = 0.0;
        double commanded = 0.0;
        double moved = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            residual  = std::max(residual, std::abs(current_pose[i] - target_pose[i]));
            commanded = std::max(commanded, std::abs(target_pose[i] - start_pose[i]));
            moved     = std::max(moved, std::abs(current_pose[i] - start_pose[i]));
        }

        bool off_target = false;
        if (!motion_looks_executed(residual, commanded, moved, step.tolerance, &off_target)) {
            return fail(
                "task space trajectory appears to have been ignored: moved " + std::to_string(moved)
                + " m of the commanded " + std::to_string(commanded) + " m");
        }
        if (off_target) {
            RCLCPP_WARN(
                node_->get_logger(), "%s finished with a residual position error of %.4f m (tolerance %.4f m)",
                state_name.c_str(), residual, step.tolerance);
        } else {
            RCLCPP_INFO(node_->get_logger(), "%s finished, position error %.4f m", state_name.c_str(), residual);
        }
    }
    return true;
}

bool TaskExecutor::run_timed_state(const Step& step, const std::string& state_name, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (!ctx_->set_exp_state(state_name)) {
        return fail("failed to switch arm_controller to " + state_name);
    }
    sleep_with_spin(node_, state_settle_);
    if (!wait_for_state(state_name, error)) {
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "%s active for %.2f s", state_name.c_str(), step.duration);
    sleep_with_spin(node_, step.duration);

    // 结束后把控制器交还给 idel：teach / measure 都只允许 idel 进出。
    if (!ctx_->set_exp_state(kIdleState)) {
        return fail("failed to switch arm_controller back to idel after " + state_name);
    }
    sleep_with_spin(node_, state_settle_);
    return wait_for_idel(state_name, error);
}

bool TaskExecutor::run_tool(const Step& step, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (tool_pub_ == nullptr) {
        return fail("tool command publisher is unavailable");
    }

    const bool on = tool_action_is_on(step.action);
    std_msgs::msg::Bool message;
    message.data = on;
    tool_pub_->publish(message);

    RCLCPP_INFO(
        node_->get_logger(), "tool '%s' -> %s (action '%s')", tool_name_.c_str(), on ? "ON" : "OFF",
        step.action.c_str());

    // 只发布不等待：开关量本身没有运动过程。需要建立时间（例如吸盘抽真空）时，
    // 由序列里紧随其后的 wait 步骤承担，这样时长由配置决定而不是写死在代码里。
    return true;
}

bool TaskExecutor::run_step(const Step& step, std::string* error) {
    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (step.type == "wait") {
        // 不切换控制器状态：上一步完成后控制器自身处于保持态（idel / *_traj 的 STOP 阶段）。
        sleep_with_spin(node_, step.duration);
        return true;
    }

    if (step.type == "joint_move") {
        return run_joint_move(step, error);
    }

    if (step.type == "joint_console") {
        return run_joint_console(step, error);
    }

    if (step.type == "cart_move") {
        return run_task_space_move(step, "cart_traj", error);
    }

    if (step.type == "admittance_move") {
        return run_task_space_move(step, "admittance", error);
    }

    if (step.type == "tool") {
        return run_tool(step, error);
    }

    if (step.type == "teach") {
        return run_timed_state(step, "teach_pendant", error);
    }

    if (step.type == "measure") {
        return run_timed_state(step, "measure", error);
    }

    if (step.type == "idel") {
        if (!ctx_->set_exp_state(kIdleState)) {
            return fail("failed to switch arm_controller to idel");
        }
        // 先等过状态刷新周期，避免读到切换前的旧值（详见 state_settle_ 的说明）。
        sleep_with_spin(node_, state_settle_);
        if (!wait_for_idel("idel", error)) {
            return false;
        }
        sleep_with_spin(node_, idel_hold_);
        return true;
    }

    if (step.type == "reset") {
        // 完成条件只看 current_state：控制器侧的 reset_settle 兜底保证它必然回到 idel，
        // 因此任务层不需要重复做关节误差判定（也不应重复声明 reset_* 参数）。
        if (!ctx_->set_exp_state("reset")) {
            return fail("failed to switch arm_controller to reset");
        }
        sleep_with_spin(node_, state_settle_);
        return wait_for_idel("reset", error);
    }

    return fail("step type '" + step.type + "' is not implemented yet");
}

bool TaskExecutor::run_sequence(const std::string& name) {
    const auto sequence = sequences_.find(name);
    if (sequence == sequences_.end()) {
        RCLCPP_ERROR(
            node_->get_logger(), "unknown sequence '%s'; available sequences: %s", name.c_str(),
            join(sequence_names()).c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "running sequence '%s' (%zu steps)", name.c_str(), sequence->second.size());

    for (std::size_t i = 0; i < sequence->second.size(); ++i) {
        const Step& step = sequence->second[i];
        const StepSpec spec = step_spec(step.type);

        // 控制器侧「只能 idel 进出」的约束由步骤类型属性驱动，不是硬编码名单。
        if (spec.requires_idel && !ctx_->require_controller_idel(step.type)) {
            RCLCPP_ERROR(
                node_->get_logger(), "%s: arm_controller is not in idel before a %s step", locate(name, i, step).c_str(),
                step.type.c_str());
            fall_back_to_idel();
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "%s: running", locate(name, i, step).c_str());

        std::string error;
        if (!run_step(step, &error)) {
            RCLCPP_ERROR(node_->get_logger(), "%s: %s", locate(name, i, step).c_str(), error.c_str());
            fall_back_to_idel();
            return false;
        }
    }

    RCLCPP_INFO(node_->get_logger(), "sequence '%s' finished", name.c_str());
    return true;
}
