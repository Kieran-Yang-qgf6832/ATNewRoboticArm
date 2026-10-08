#include "task_executor.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
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

}  // namespace

TaskExecutor::TaskExecutor(const rclcpp::Node::SharedPtr& node, TaskContext* ctx)
    : node_(node)
    , ctx_(ctx) {
    if (node_ != nullptr) {
        step_timeout_    = node_->declare_parameter<double>("step_timeout", step_timeout_);
        idel_hold_       = node_->declare_parameter<double>("idel_hold", idel_hold_);
        state_settle_    = node_->declare_parameter<double>("state_settle", state_settle_);
        settle_duration_ = node_->declare_parameter<double>("settle_duration", settle_duration_);
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

    // 控制器若因求解失败回退到 idel，说明这次运动没有走完，必须上报。
    const std::string state = ctx_->controller_state(2.0);
    if (state != "joint_traj") {
        return fail(
            "arm_controller left joint_traj (now '" + (state.empty() ? std::string("<unknown>") : state)
            + "'), likely a solve failure");
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

    const std::string state = ctx_->controller_state(2.0);
    if (state != state_name) {
        return fail(
            "arm_controller left " + state_name + " (now '" + (state.empty() ? std::string("<unknown>") : state)
            + "'), likely a solve failure");
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

    if (step.type == "cart_move") {
        return run_task_space_move(step, "cart_traj", error);
    }

    if (step.type == "admittance_move") {
        return run_task_space_move(step, "admittance", error);
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
