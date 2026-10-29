#include "arm_fsm.hpp"

#include "arm_fsm_factory.hpp"
#include "trajectory.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>

#include <rclcpp/duration.hpp>
#include <rclcpp/rclcpp.hpp>

#include <robot_msgs/msg/vector.hpp>

namespace {

constexpr int kMaxArmTrajectoryPoints = 8;

const std::string kIdelStateName = "idel";

// load_trajectory_from_message 依赖各 FSM 的 traj 预分配容量不小于最大点数，
// 在订阅回调中复用预分配 buffer（clear + add_point），避免实时路径内重新分配内存。
int model_dof_from_context(std::any ctx) {
    auto* factory = std::any_cast<FSMArmControlFactory*>(ctx);
    if (factory == nullptr || factory->model_ == nullptr) {
        return 0;
    }
    return factory->model_->dof();
}

/**
 * @brief 用 RNEA 在 dq = ddq = 0 下算出重力补偿力矩
 *
 * reset / idel 仅靠 kp 保持位置时，重力静差为 τ_gravity / kp。零位姿下 joint4 的
 * τ 约 1.24 N·m 而 default_kp 只有 10，静差约 0.124 rad，远超 reset_tolerance
 * （0.01 rad），于是复位永远不达标并静默回退 idel（docs/bugs.md O6）。
 *
 * 把重力力矩作为前馈直接补掉，就不必为了压静差把 kp 加到可能振荡的程度。
 * 这与 teach_pendant 用的是同一套机制（同一份模型、同一个 torque 前馈字段）。
 *
 * @note 只读写调用方预分配的 buffer，不做任何内存分配，可在实时路径调用。
 * @return 计算成功返回 true；模型不可用、抛异常或结果非有限值时返回 false，
 *         调用方应退回纯 kp 保持（会有静差，但不会失控）
 */
bool compute_gravity_torque(FSMArmControlFactory* factory,
                            std::size_t joint_count,
                            Eigen::VectorXd* joint_pos,
                            const Eigen::VectorXd& zero,
                            Eigen::VectorXd* gravity_torque,
                            const char* state_name) {
    const auto count = static_cast<Eigen::Index>(joint_count);
    if (factory == nullptr || factory->model_ == nullptr || factory->node_ == nullptr || joint_pos == nullptr
        || gravity_torque == nullptr || joint_pos->size() != count || zero.size() != count
        || gravity_torque->size() != count || factory->state_.size() != joint_count) {
        return false;
    }

    for (std::size_t i = 0; i < joint_count; ++i) {
        (*joint_pos)(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        // dq = ddq = 0 时 RNEA 退化为纯重力力矩。
        if (!factory->model_->inverse_dynamic(*joint_pos, zero, zero, gravity_torque)) {
            RCLCPP_WARN_THROTTLE(
                factory->node_->get_logger(), *factory->node_->get_clock(), 2000,
                "%s: 重力补偿求解失败，本次退回纯 kp 保持（会有重力静差）", state_name);
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 2000,
            "%s: 重力补偿抛出异常：%s，本次退回纯 kp 保持", state_name, error.what());
        return false;
    }

    if (!gravity_torque->allFinite()) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 2000,
            "%s: 重力补偿结果非有限值，本次退回纯 kp 保持", state_name);
        return false;
    }

    return true;
}

/**
 * @brief 锁存当前实测关节角，作为后续「保持」的位置指令
 *
 * 必须在**进入保持时调用一次**。绝不能每个周期重新读取实测值：那样位置指令永远
 * 等于当前值，位置误差恒为 0，kp 再大也产生不了恢复力，机械臂只剩 kd 阻尼，会在
 * 重力下缓慢坠落——"保持"名不副实。IDELState 用的就是这种锁存式保持。
 *
 * @param factory       状态机工厂
 * @param joint_count   关节数
 * @param hold_position 输出：预分配的保持位置缓存
 */
void latch_joint_position(
    FSMArmControlFactory* factory, std::size_t joint_count, std::vector<float>* hold_position) {
    if (factory == nullptr || hold_position == nullptr || factory->state_.size() != joint_count
        || hold_position->size() != joint_count) {
        return;
    }

    for (std::size_t i = 0; i < joint_count; ++i) {
        (*hold_position)[i] = factory->state_[i].position;
    }
}

/**
 * @brief 按锁存值下发保持命令
 *
 * 位置指令取锁存值（固定不变），kp 才会产生真实的恢复力。
 *
 * 同时复用重力前馈（docs/bugs.md O7）：只写 torque = 0 时，进入 cart_traj /
 * joint_traj / admittance 后等待轨迹到达的这段时间里，手臂只剩 kp 的刚度，joint4
 * 会按 τ/kp 塌陷约 0.089 rad（5.1°）——笛卡尔运动的起点因此不是指令要求的位姿，
 * 而且没有任何提示。O6 只给 reset / idel 补了前馈，这里补上同一根因的另一条路径。
 *
 * @param factory       状态机工厂
 * @param joint_count   关节数
 * @param hold_position 锁存的目标位置
 * @param default_kp    位置增益
 * @param default_kd    速度增益
 * @param gravity       保持路径共用的重力前馈；nullptr 表示不做补偿（torque = 0）
 * @param state_name    状态名，仅用于补偿失败的日志
 */
void write_hold_command(
    FSMArmControlFactory* factory, std::size_t joint_count, const std::vector<float>& hold_position,
    const std::vector<float>& default_kp, const std::vector<float>& default_kd, HoldGravityFeedforward* gravity,
    const char* state_name) {
    if (factory == nullptr || factory->command_.size() != joint_count || hold_position.size() != joint_count
        || default_kp.size() != joint_count || default_kd.size() != joint_count) {
        return;
    }

    const bool gravity_ok = gravity != nullptr && gravity->refresh(factory, joint_count, state_name);

    for (std::size_t i = 0; i < joint_count; ++i) {
        auto& command    = factory->command_[i];
        command.position = hold_position[i];
        command.velocity = 0.0f;
        command.torque   = gravity_ok ? gravity->torque(i) : 0.0f;
        command.kp       = default_kp[i];
        command.kd       = default_kd[i];
        command.ki       = 0.0f;
    }
}

void load_default_gains(
    FSMArmControlFactory* factory, std::size_t joint_count, std::vector<double>* default_kp_param, std::vector<double>* default_kd_param,
    std::vector<float>* default_kp, std::vector<float>* default_kd) {
    if (factory == nullptr || factory->node_ == nullptr || default_kp_param == nullptr || default_kd_param == nullptr
        || default_kp == nullptr || default_kd == nullptr) {
        return;
    }

    factory->node_->get_parameter("default_kp", *default_kp_param);
    factory->node_->get_parameter("default_kd", *default_kd_param);

    const std::size_t param_capacity = std::max({joint_count, default_kp_param->size(), default_kd_param->size()});
    default_kp_param->reserve(param_capacity);
    default_kd_param->reserve(param_capacity);
    default_kp->resize(joint_count);
    default_kd->resize(joint_count);
    for (std::size_t i = 0; i < joint_count; ++i) {
        (*default_kp)[i] = i < default_kp_param->size() ? static_cast<float>((*default_kp_param)[i]) : 0.0f;
        (*default_kd)[i] = i < default_kd_param->size() ? static_cast<float>((*default_kd_param)[i]) : 0.0f;
    }
}

bool trajectory_finished(const Trajectory& traj, const rclcpp::Time& start_time, const rclcpp::Time& time) {
    // points 是预分配的定长池，不能直接取 points.back()/points.empty()；
    // 必须通过 Trajectory 的 size()/back() 接口按实际已加入的点数（index）判断。
    return !traj.empty() && (time - start_time) >= traj.back().time;
}

bool load_trajectory_from_message(
    const std::vector<robot_msgs::msg::Vector>& points, const std::vector<float>& seconds, std::size_t dof,
    Trajectory* traj) {
    if (traj == nullptr || points.empty() || seconds.size() != points.size() || points.size() > traj->capacity()) {
        return false;
    }

    for (const auto& point : points) {
        if (point.pos.size() != dof || (!point.vel.empty() && point.vel.size() != dof)
            || (!point.acc.empty() && point.acc.size() != dof)) {
            return false;
        }
    }

    traj->clear();
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (seconds[i] < 0.0f || !std::isfinite(seconds[i])) {
            return false;
        }
        if (!traj->add_point(
                points[i].pos, points[i].vel, points[i].acc,
                rclcpp::Duration(std::chrono::duration<double>(seconds[i])))) {
            return false;
        }
    }
    return !traj->empty();
}

} // namespace

void HoldGravityFeedforward::initialize(FSMArmControlFactory* factory, std::size_t joint_count, const char* state_name) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    // 只在构造阶段读参数与分配内存，run() 里不再碰参数（实时约束）。
    bool gravity_compensation_param       = true;
    const bool gravity_compensation_found = factory->node_->get_parameter("gravity_compensation", gravity_compensation_param);
    enabled_                              = gravity_compensation_param;

    double gravity_scale_param = 1.0;
    factory->node_->get_parameter("gravity_compensation_scale", gravity_scale_param);
    // 参数回调已经拒绝越界值；这里再夹一次，防止直接改 YAML 绕过校验。
    scale_ = std::clamp(gravity_scale_param, 0.0, 1.0);

    RCLCPP_INFO(
        factory->node_->get_logger(), "%s: gravity_compensation=%s scale=%.3f (param found=%s)", state_name != nullptr ? state_name : "?",
        enabled_ ? "true" : "false", scale_, gravity_compensation_found ? "yes" : "no");

    const auto dof = static_cast<Eigen::Index>(joint_count);
    joint_pos_.resize(dof);
    zero_.setZero(dof);
    torque_.resize(dof);
    available_ = false;
}

bool HoldGravityFeedforward::refresh(FSMArmControlFactory* factory, std::size_t joint_count, const char* state_name) {
    if (!enabled_ || factory == nullptr) {
        available_ = false;
        return false;
    }
    available_ = compute_gravity_torque(factory, joint_count, &joint_pos_, zero_, &torque_, state_name);
    return available_;
}

float HoldGravityFeedforward::torque(std::size_t index) const {
    if (!available_ || index >= static_cast<std::size_t>(torque_.size())) {
        return 0.0f;
    }
    return static_cast<float>(torque_(static_cast<Eigen::Index>(index)) * scale_);
}

void AdmittanceState::load_admittance_parameters(const char* name, const Eigen::VectorXd& fallback, Eigen::VectorXd& destination) {
    if (factory == nullptr || factory->node_ == nullptr || name == nullptr || destination.size() != fallback.size()) {
        return;
    }

    std::vector<double> values;
    factory->node_->get_parameter(name, values);
    for (Eigen::Index i = 0; i < destination.size(); ++i) {
        destination(i) = i < static_cast<Eigen::Index>(values.size()) ? values[static_cast<std::size_t>(i)] : fallback(i);
    }
}

IDELState::IDELState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity = std::max({joint_count_, default_kp_param_.size(), default_kd_param_.size()});
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    hold_position_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    // 重力补偿：只在构造阶段读参数与分配内存，run() 里不再碰参数（实时约束）。
    gravity_.initialize(factory, joint_count_, name.c_str());
}

bool IDELState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    // 重力前馈：只靠 kp 保持时静差为 τ/kp，零位姿下 joint4 会偏出 0.1 rad 以上（O6）。
    gravity_.refresh(factory, joint_count_, fsm_name_.c_str());

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
        command.position  = hold_position_[i];
        command.velocity  = 0.0f;
        command.torque    = gravity_.torque(i);
        command.kp        = default_kp_[i];
        command.kd        = default_kd_[i];
        command.ki        = 0.0f;                                // 只要i的值改变，底层会立即清空积分项
    }

    return true;
}

bool IDELState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

const std::string& IDELState::check_switch() const {
    if (factory == nullptr || factory->exp_state_name.empty()) { // 允许切换到任何状态
        return fsm_name_;
    }

    return factory->exp_state_name;
}

bool IDELState::run(const rclcpp::Time& time) {
    (void)time;

    if (factory == nullptr || factory->command_.size() != joint_count_) {
        return false;
    }

    // 重力前馈：只靠 kp 保持时静差为 τ/kp，零位姿下 joint4 会偏出 0.1 rad 以上（O6）。
    gravity_.refresh(factory, joint_count_, fsm_name_.c_str());

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        auto& command    = factory->command_[i];
        command.position = hold_position_[i];
        command.velocity = 0.0f;
        command.torque   = gravity_.torque(i);
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    return true;
}

ResetState::ResetState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("reset_joint_pos", reset_joint_pos_param_);
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity =
        std::max({joint_count_, reset_joint_pos_param_.size(), default_kp_param_.size(), default_kd_param_.size()});
    reset_joint_pos_param_.reserve(param_capacity);
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    start_joint_pos_.resize(joint_count_);
    reset_joint_pos_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
    double reset_duration_param  = 3.0;
    double reset_tolerance_param = 0.01;
    double reset_settle_param    = 2.0;
    factory->node_->get_parameter("reset_duration", reset_duration_param);
    factory->node_->get_parameter("reset_tolerance", reset_tolerance_param);
    factory->node_->get_parameter("reset_settle", reset_settle_param);
    reset_duration_  = reset_duration_param > 0.0 ? static_cast<float>(reset_duration_param) : 1.0f;
    reset_tolerance_ = reset_tolerance_param >= 0.0 ? static_cast<float>(reset_tolerance_param) : 0.01f;
    reset_settle_    = reset_settle_param >= 0.0 ? static_cast<float>(reset_settle_param) : 0.0f;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        reset_joint_pos_[i] = i < reset_joint_pos_param_.size() ? static_cast<float>(reset_joint_pos_param_[i]) : 0.0f;
        default_kp_[i]      = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i]      = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    // 重力补偿：只在构造阶段读参数与分配内存，run() 里不再碰参数（实时约束）。
    gravity_.initialize(factory, joint_count_, name.c_str());
}

bool ResetState::enter(const std::string& last_state, const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < start_joint_pos_.size(); ++i) {
        start_joint_pos_[i] = factory->state_[i].position;
    }

    reset_start_time_ = time;
    progress_         = 0.0f;
    reset_done_       = joint_count_ == 0;
    give_up_warned_   = false;
    return true;
}

bool ResetState::exit(const std::string& next_state) {
    if (factory != nullptr && next_state == "idel") {
        factory->exp_state_name = "idel";
    }
    return next_state == "idel";
}

const std::string& ResetState::check_switch() const {
    if (reset_done_) {
        return kIdelStateName;
    }
    return fsm_name_;
}

bool ResetState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    const float elapsed_s = static_cast<float>((time - reset_start_time_).seconds());
    progress_             = std::min(1.0f, elapsed_s / reset_duration_);

    bool joint_reached = true;
    float max_error    = 0.0f;
    std::size_t worst  = 0;
    // 重力前馈：否则零位姿下 joint4 的静差约 0.124 rad，远超 reset_tolerance，
    // 复位永远不达标，只能静默回退 idel（docs/bugs.md O6）。
    gravity_.refresh(factory, joint_count_, fsm_name_.c_str());
    for (std::size_t i = 0; i < joint_count_; ++i) {
        const float delta          = reset_joint_pos_[i] - start_joint_pos_[i];
        auto& command              = factory->command_[i];
        command.position           = start_joint_pos_[i] + delta * progress_;
        command.velocity           = progress_ < 1.0f ? delta / reset_duration_ : 0.0f;
        command.torque             = gravity_.torque(i);
        command.kp                 = default_kp_[i];
        command.kd                 = default_kd_[i];
        command.ki                 = 0.0f;
        const float position_error = std::fabs(factory->state_[i].position - reset_joint_pos_[i]);
        joint_reached              = joint_reached && position_error <= reset_tolerance_;
        if (position_error > max_error) {
            max_error = position_error;
            worst     = i;
        }
    }

    if (progress_ >= 1.0f && joint_reached) {
        reset_done_ = true;
        return true;
    }

    // 兜底：复位时长结束后再宽限 reset_settle_ 秒仍未全部到位（例如重力静差超过容差），
    // 打一次 WARN 并强制切回 idel，避免 FSM 卡死在 reset 后静默吞掉所有 exp_state 请求。
    if (!reset_done_ && !give_up_warned_ && elapsed_s >= reset_duration_ + reset_settle_) {
        give_up_warned_ = true;
        RCLCPP_WARN(
            factory->node_->get_logger(),
            "reset did not reach the target within %.2f s (reset_duration=%.2f s, reset_settle=%.2f s): max joint "
            "error %.4f rad on joint%zu (tolerance %.4f rad); forcing the switch to idel, check reset_tolerance / "
            "default_kp (gravity sag) if the offset persists",
            elapsed_s, static_cast<double>(reset_duration_), static_cast<double>(reset_settle_),
            static_cast<double>(max_error), worst + 1, static_cast<double>(reset_tolerance_));
        reset_done_ = true;
    }
    return true;
}

CartTrajState::CartTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , point(6)
    , traj(6, static_cast<int>(kMaxArmTrajectoryPoints))
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    joint_count_ = model_dof_from_context(ctx);
    task_dof_    = 6;
    joint_pos_.resize(static_cast<Eigen::Index>(joint_count_));
    torque_.resize(static_cast<Eigen::Index>(joint_count_));
    task_force_.setZero(static_cast<Eigen::Index>(task_dof_));
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);
    hold_position_.resize(joint_count_);
    gravity_.initialize(factory, joint_count_, name.c_str());
    precheck_target_.resize(static_cast<Eigen::Index>(task_dof_));
    precheck_seed_.resize(static_cast<Eigen::Index>(joint_count_));
    precheck_solution_.resize(static_cast<Eigen::Index>(joint_count_));

    cart_traj_cmd_sub_ = factory->node_->create_subscription<robot_msgs::msg::CartTrajCmd>(
        "arm_cart_traj", rclcpp::QoS(10), [this](const robot_msgs::msg::CartTrajCmd& msg) {
            if (state != TrajPhase::STOP || !load_trajectory_from_message(msg.position, msg.seconds, task_dof_, &traj)) {
                return;
            }
            // 记下终点供 run() 做可达性预检。这里只做拷贝，不开新轨迹：预检失败时
            // 必须保证一个运动指令都没下发过。
            const auto& final_waypoint = msg.position.back().pos;
            for (Eigen::Index row = 0; row < precheck_target_.size(); ++row) {
                precheck_target_(row) = final_waypoint[static_cast<std::size_t>(row)];
            }
            precheck_pending_ = true;

            const auto now = factory->node_->get_clock()->now();
            traj.start(now);
            traj_start_time_ = now;
            state            = TrajPhase::MOVING;
        });
}

bool CartTrajState::check_target_reachable(double* residual) {
    if (factory == nullptr || factory->arm_solve_ == nullptr || residual == nullptr) {
        return false;
    }

    // 用当前实测关节角作初值：终点通常离当前位姿不远，这个初值比零位形好得多。
    for (std::size_t i = 0; i < joint_count_; ++i) {
        precheck_seed_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    precheck_solution_ = precheck_seed_;
    if (!factory->arm_solve_->inverse_kinamic(precheck_target_, &precheck_solution_)
        || !precheck_solution_.allFinite()) {
        *residual = factory->arm_solve_->last_ik_error_norm();
        return false;
    }

    *residual = 0.0;
    return true;
}

bool CartTrajState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || default_kp_.size() != joint_count_ || default_kd_.size() != joint_count_) {
        return false;
    }

    state = TrajPhase::STOP;
    traj.stop();
    // 进入时锁存一次当前位置。保持命令必须基于固定目标：若每个周期重新读取实测值，
    // 位置误差恒为 0，kp 再大也不产生恢复力，机械臂会在重力下缓慢坠落。
    latch_joint_position(factory, joint_count_, &hold_position_);
    write_hold_command(factory, joint_count_, hold_position_, default_kp_, default_kd_, &gravity_, fsm_name_.c_str());
    return true;
}

bool CartTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    // 清掉未消费的预检请求：否则下一次进入本状态时会拿旧轨迹的途经点做预检。
    precheck_pending_ = false;
    return true;
}

const std::string& CartTrajState::check_switch() const {
    if (factory == nullptr || factory->exp_state_name.empty() || factory->exp_state_name == fsm_name_) {
        return fsm_name_;
    }
    // 运动中允许被 idel 打断：exit() 会停掉轨迹，IDELState::enter() 再锁存当前位姿并
    // 保持——这就是「停止」的语义（tasks 的 joint_console stop 依赖它）。
    // 其余目标仍要求轨迹走完（STOP）才切换，避免中途直接跳进另一条轨迹。
    if (state == TrajPhase::STOP || factory->exp_state_name == kIdelStateName) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool CartTrajState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (state == TrajPhase::STOP) {
        write_hold_command(factory, joint_count_, hold_position_, default_kp_, default_kd_, &gravity_, fsm_name_.c_str());
        return true;
    }

    // 下发前可达性预检（docs/bugs.md O2）：在**下发任何运动指令之前**确认整条轨迹可达。
    // 不做这一步时，手臂会先沿轨迹一路走到工作空间边界（实测末端走约 0.40 m），
    // 再因逆解失败中止——那时运动已经发生，安全语义上已经晚了。
    if (precheck_pending_) {
        precheck_pending_ = false;

        double residual = 0.0;
        if (!check_target_reachable(&residual)) {
            state = TrajPhase::STOP;
            traj.stop();
            set_fault_stage("Cartesian target reachability pre-check");

            if (factory->arm_solve_->last_ik_result() == IKSolver::SolveResult::Unreachable) {
                // 这是「目标不可达」，不是「求解器坏了」：报出最近可达位姿离终点还有多远，
                // 让操作者知道该改目标，而不是去查逆解器。
                RCLCPP_ERROR(
                    factory->node_->get_logger(),
                    "Cartesian trajectory REJECTED before any motion: the target is UNREACHABLE — the nearest reachable "
                    "pose is still %.6f away from it (IK tolerance is %.1e). Without this pre-check the arm would have "
                    "walked to the edge of its workspace and only then aborted. Fix the target, not the solver.",
                    residual, IKSolver::tuning().tolerance);
            } else {
                RCLCPP_ERROR(
                    factory->node_->get_logger(),
                    "Cartesian trajectory REJECTED before any motion: inverse kinematics failed numerically "
                    "(residual %.6f); reachability is unknown.",
                    residual);
            }
            return false;
        }
    }

    traj.update(time, point);
    if (point.pos.size() != static_cast<Eigen::Index>(task_dof_) || point.vel.size() != static_cast<Eigen::Index>(task_dof_)
        || point.acc.size() != static_cast<Eigen::Index>(task_dof_) || !point.pos.allFinite() || !point.vel.allFinite()
        || !point.acc.allFinite()) {
        state = TrajPhase::STOP;
        traj.stop();
        set_fault_stage("Cartesian trajectory sampling");
        return false;
    }

    // 用当前实测关节角作为逆解初值：轨迹首点就是当前位姿，从这个种子出发误差近似为 0，
    // 后续点也是局部小范围变化，避免从零位形起步导致的迭代不收敛。
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_kinamic(point.pos, &joint_pos_)
            || joint_pos_.size() != static_cast<Eigen::Index>(joint_count_) || !joint_pos_.allFinite()) {
            state = TrajPhase::STOP;
            traj.stop();
            set_fault_stage("Cartesian trajectory inverse kinematics");
            if (joint_count_ == 6) {
                // 打印「目标」与「当前实测关节角的正解」以及二者的任务空间误差范数，
                // 用于区分「目标不可达」与「迭代不收敛」。
                Eigen::VectorXd seed_joints(6);
                for (std::size_t i = 0; i < 6; ++i) {
                    seed_joints(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
                }
                Eigen::VectorXd measured_task(6);
                double error_norm = -1.0;
                if (factory->arm_solve_->forward_kinamic(seed_joints, &measured_task)) {
                    Eigen::VectorXd task_error(6);
                    if (factory->task_map_->error_map(point.pos, measured_task, &task_error)) {
                        error_norm = task_error.norm();
                    }
                }
                RCLCPP_WARN_THROTTLE(
                    factory->node_->get_logger(), *factory->node_->get_clock(), 1000,
                    "Cartesian IK failed at t=%.3f s: %s, ik_err=%.3e (iters=%d), tracking_err=%.6f\n"
                    "  target (9dp) = [%.9f, %.9f, %.9f, %.9f, %.9f, %.9f]\n"
                    "  seed   (9dp) = [%.9f, %.9f, %.9f, %.9f, %.9f, %.9f]\n"
                    "  fk(seed)     = [%.4f %.4f %.4f | %.4f %.4f %.4f]",
                    (time - traj_start_time_).seconds(),
                    factory->arm_solve_->last_ik_result() == IKSolver::SolveResult::Unreachable
                        ? "target is UNREACHABLE (iteration stalled at the edge of the reachable set)"
                        : "solver failed numerically (reachability unknown)",
                    factory->arm_solve_->last_ik_error_norm(),
                    factory->arm_solve_->last_ik_iterations(), error_norm, point.pos(0), point.pos(1), point.pos(2),
                    point.pos(3), point.pos(4), point.pos(5), seed_joints(0), seed_joints(1), seed_joints(2),
                    seed_joints(3), seed_joints(4), seed_joints(5), measured_task(0), measured_task(1), measured_task(2),
                    measured_task(3), measured_task(4), measured_task(5));
            }
            return false;
        }
        if (!factory->arm_solve_->inverse_dynamic(joint_pos_, point.vel, point.acc, task_force_, &torque_)
            || torque_.size() != static_cast<Eigen::Index>(joint_count_) || !torque_.allFinite()) {
            state = TrajPhase::STOP;
            traj.stop();
            set_fault_stage("Cartesian trajectory inverse dynamics");
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Cartesian trajectory solve failed: %s", error.what());
        state = TrajPhase::STOP;
        traj.stop();
        set_fault_stage("Cartesian trajectory solve threw");
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        auto& command    = factory->command_[i];
        command.position = static_cast<float>(joint_pos_(index));
        command.velocity = 0.0f;
        command.torque   = static_cast<float>(torque_(index));
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    if (trajectory_finished(traj, traj_start_time_, time)) {
        traj.stop();
        state = TrajPhase::STOP;
        // 轨迹结束时锁存终点：否则进入 STOP 后位置指令会跟着实测值跑，失去刚度。
        latch_joint_position(factory, joint_count_, &hold_position_);
    }

    return true;
}

JointTrajState::JointTrajState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , point(model_dof_from_context(ctx))
    , traj(model_dof_from_context(ctx), static_cast<int>(kMaxArmTrajectoryPoints))
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    joint_count_ = model_dof_from_context(ctx);
    torque.resize(static_cast<Eigen::Index>(joint_count_));
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);
    hold_position_.resize(joint_count_);
    gravity_.initialize(factory, joint_count_, name.c_str());

    joint_trajectory_cmd_sub_ = factory->node_->create_subscription<robot_msgs::msg::JointTrajCmd>(
        "arm_joint_traj", rclcpp::QoS(10), [this](const robot_msgs::msg::JointTrajCmd& msg) {
            if (state != TrajPhase::STOP || !load_trajectory_from_message(msg.position, msg.seconds, joint_count_, &traj)) {
                return;
            }
            const auto now = factory->node_->get_clock()->now();
            traj.start(now);
            traj_start_time_ = now;
            state            = TrajPhase::MOVING;
        });
}

bool JointTrajState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->model_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || default_kp_.size() != joint_count_ || default_kd_.size() != joint_count_) {
        return false;
    }

    state = TrajPhase::STOP;
    traj.stop();
    // 进入时锁存一次当前位置（理由同 CartTrajState::enter）。
    latch_joint_position(factory, joint_count_, &hold_position_);
    write_hold_command(factory, joint_count_, hold_position_, default_kp_, default_kd_, &gravity_, fsm_name_.c_str());
    return true;
}

bool JointTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    return true;
}

const std::string& JointTrajState::check_switch() const {
    if (factory == nullptr || factory->exp_state_name.empty() || factory->exp_state_name == fsm_name_) {
        return fsm_name_;
    }
    // 运动中允许被 idel 打断：exit() 会停掉轨迹，IDELState::enter() 再锁存当前位姿并
    // 保持——这就是「停止」的语义（tasks 的 joint_console stop 依赖它）。
    // 其余目标仍要求轨迹走完（STOP）才切换，避免中途直接跳进另一条轨迹。
    if (state == TrajPhase::STOP || factory->exp_state_name == kIdelStateName) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool JointTrajState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || factory->model_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (state == TrajPhase::STOP) {
        write_hold_command(factory, joint_count_, hold_position_, default_kp_, default_kd_, &gravity_, fsm_name_.c_str());
        return true;
    } else if (state == TrajPhase::MOVING) {
        traj.update(time, point);
        if (point.pos.size() != static_cast<Eigen::Index>(joint_count_) || point.vel.size() != static_cast<Eigen::Index>(joint_count_)
            || point.acc.size() != static_cast<Eigen::Index>(joint_count_) || !point.pos.allFinite() || !point.vel.allFinite()
            || !point.acc.allFinite()) {
            state = TrajPhase::STOP;
            traj.stop();
            set_fault_stage("Joint trajectory sampling");
            return false;
        }

        try {
            if (!factory->model_->inverse_dynamic(point.pos, point.vel, point.acc, &torque)
                || torque.size() != static_cast<Eigen::Index>(joint_count_) || !torque.allFinite()) {
                state = TrajPhase::STOP;
                traj.stop();
                set_fault_stage("Joint trajectory inverse dynamics");
                return false;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Joint trajectory inverse dynamics failed: %s",
                error.what());
            state = TrajPhase::STOP;
            traj.stop();
            set_fault_stage("Joint trajectory inverse dynamics");
            return false;
        }

        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index = static_cast<Eigen::Index>(i);
            auto& command    = factory->command_[i];
            command.position = static_cast<float>(point.pos(index));
            command.velocity = static_cast<float>(point.vel(index));
            command.torque   = static_cast<float>(torque(index));
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if (trajectory_finished(traj, traj_start_time_, time)) {
            traj.stop();
            state = TrajPhase::STOP;
            // 轨迹结束时锁存终点（理由同 CartTrajState::run）。
            latch_joint_position(factory, joint_count_, &hold_position_);
        }
    }
    return true;
}

ServoState::ServoState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    joint_count_ = model_dof_from_context(ctx);
    task_dof_    = 6;
    joint_pos_.resize(static_cast<Eigen::Index>(joint_count_));
    joint_velocity_.resize(static_cast<Eigen::Index>(joint_count_));
    task_position_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_position_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_velocity_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_acceleration_.resize(static_cast<Eigen::Index>(task_dof_));
    task_force_.setZero(static_cast<Eigen::Index>(task_dof_));
    torque_.resize(static_cast<Eigen::Index>(joint_count_));
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);

    servo_ctrl_cmd_sub_ = factory->node_->create_subscription<robot_msgs::msg::ServoCtrlCmd>(
        "arm_servo", rclcpp::QoS(10), [this](const robot_msgs::msg::ServoCtrlCmd& msg) {
            // Vector.pos 承载该命令分量的数值，vel/acc 为轨迹点导数，伺服为单条控制指令只读取 pos。
            if (msg.velocity.pos.size() != task_dof_) {
                return;
            }
            for (std::size_t i = 0; i < task_dof_; ++i) {
                desired_task_velocity_(static_cast<Eigen::Index>(i)) = msg.velocity.pos[i];
            }
            desired_task_acceleration_.setZero();
            if (msg.acceleration.pos.size() == task_dof_) {
                for (std::size_t i = 0; i < task_dof_; ++i) {
                    desired_task_acceleration_(static_cast<Eigen::Index>(i)) = msg.acceleration.pos[i];
                }
            }
            task_force_.setZero();
            if (msg.force.pos.size() == task_dof_) {
                for (std::size_t i = 0; i < task_dof_; ++i) {
                    task_force_(static_cast<Eigen::Index>(i)) = msg.force.pos[i];
                }
            }
        });
}

bool ServoState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;

    if (factory == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }
    try {
        if (!factory->arm_solve_->forward_kinamic(joint_pos_, &task_position_)
            || task_position_.size() != static_cast<Eigen::Index>(task_dof_) || !task_position_.allFinite()) {
            return false;
        }
    } catch (const std::exception&) {
        return false;
    }

    desired_task_position_ = task_position_;
    desired_task_velocity_.setZero();
    desired_task_acceleration_.setZero();
    task_force_.setZero();
    last_update_time_ = time;
    return true;
}

bool ServoState::exit(const std::string& next_state) {
    (void)next_state;
    return true;
}

const std::string& ServoState::check_switch() const {
    if (factory != nullptr && !factory->exp_state_name.empty() && factory->exp_state_name != fsm_name_) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool ServoState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    double dt         = (time - last_update_time_).seconds();
    last_update_time_ = time;
    if (!std::isfinite(dt) || dt < 0.0) {
        dt = 0.0;
    }

    desired_task_position_.noalias() += desired_task_velocity_ * dt;
    if (!desired_task_position_.allFinite()) {
        set_fault_stage("Servo target integration");
        return false;
    }

    // 用当前实测关节角作为逆解初值，保证迭代从离目标最近的位形出发。
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_kinamic(desired_task_position_, &joint_pos_)
            || !factory->arm_solve_->inverse_velocity(joint_pos_, desired_task_velocity_, &joint_velocity_)
            || !factory->arm_solve_->inverse_dynamic(
                joint_pos_, desired_task_velocity_, desired_task_acceleration_, task_force_, &torque_)) {
            set_fault_stage("Servo solve");
            return false;
        }
    } catch (const std::exception&) {
        set_fault_stage("Servo solve");
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        auto& command    = factory->command_[i];
        command.position = static_cast<float>(joint_pos_(index));
        command.velocity = static_cast<float>(joint_velocity_(index));
        command.torque   = static_cast<float>(torque_(index));
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }
    return true;
}

AdmittanceState::AdmittanceState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx))
    , traj(6, static_cast<int>(kMaxArmTrajectoryPoints))
    , point(6) {
    joint_count_ = model_dof_from_context(ctx);
    task_dof_    = 6;
    joint_pos_.resize(static_cast<Eigen::Index>(joint_count_));
    joint_velocity_.resize(static_cast<Eigen::Index>(joint_count_));
    joint_acceleration_.setZero(static_cast<Eigen::Index>(joint_count_));
    model_torque_.resize(static_cast<Eigen::Index>(joint_count_));
    torque_residual_.resize(static_cast<Eigen::Index>(joint_count_));
    task_force_.setZero(static_cast<Eigen::Index>(task_dof_));
    task_position_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_position_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_velocity_.resize(static_cast<Eigen::Index>(task_dof_));
    desired_task_acceleration_.resize(static_cast<Eigen::Index>(task_dof_));
    position_error_.resize(static_cast<Eigen::Index>(task_dof_));
    velocity_error_.resize(static_cast<Eigen::Index>(task_dof_));
    torque_.resize(static_cast<Eigen::Index>(joint_count_));
    hold_position_.resize(joint_count_);
    gravity_.initialize(factory, joint_count_, name.c_str());
    admittance_mass_.resize(static_cast<Eigen::Index>(task_dof_));
    admittance_damping_.resize(static_cast<Eigen::Index>(task_dof_));
    admittance_stiffness_.resize(static_cast<Eigen::Index>(task_dof_));
    admittance_mass_.setOnes();
    admittance_damping_.setConstant(20.0);
    admittance_stiffness_.setZero();
    load_default_gains(factory, joint_count_, &default_kp_param_, &default_kd_param_, &default_kp_, &default_kd_);

    Eigen::VectorXd default_mass      = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(task_dof_));
    Eigen::VectorXd default_damping   = Eigen::VectorXd::Constant(static_cast<Eigen::Index>(task_dof_), 20.0);
    Eigen::VectorXd default_stiffness = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(task_dof_));
    load_admittance_parameters("admittance_mass", default_mass, admittance_mass_);
    load_admittance_parameters("admittance_damping", default_damping, admittance_damping_);
    load_admittance_parameters("admittance_stiffness", default_stiffness, admittance_stiffness_);

    admittance_cmd_sub_ = factory->node_->create_subscription<robot_msgs::msg::AdmittanceCmd>(
        "arm_admittance", rclcpp::QoS(10), [this](const robot_msgs::msg::AdmittanceCmd& msg) {
            if (trajectory_active_ || !load_trajectory_from_message(msg.position, msg.seconds, task_dof_, &traj)) {
                return;
            }
            const auto now = factory->node_->get_clock()->now();
            traj.start(now);
            traj_start_time_         = now;
            last_update_time_        = now;
            desired_task_position_     = traj.front().pos;
            desired_task_velocity_     = traj.front().vel;
            desired_task_acceleration_ = traj.front().acc;
            trajectory_active_         = true;
        });
}

bool AdmittanceState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;

    if (factory == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i))      = factory->state_[i].position;
        joint_velocity_(static_cast<Eigen::Index>(i)) = factory->state_[i].velocity;
    }
    try {
        if (!factory->arm_solve_->forward_kinamic(joint_pos_, &task_position_)
            || task_position_.size() != static_cast<Eigen::Index>(task_dof_) || !task_position_.allFinite()) {
            return false;
        }
    } catch (const std::exception&) {
        return false;
    }

    traj.stop();
    trajectory_active_     = false;
    desired_task_position_ = task_position_;
    desired_task_velocity_.setZero();
    desired_task_acceleration_.setZero();
    task_force_.setZero();
    last_update_time_ = time;
    // 轨迹未激活时按锁存值保持；进入时锁存一次当前位置（理由同 CartTrajState::enter）。
    latch_joint_position(factory, joint_count_, &hold_position_);
    return true;
}

bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
    // 必须清理：check_switch() 的切换前提是 !trajectory_active_，若离开时不清零，
    // 再次进入本状态或经由失败路径切换时会被永久挡住。
    traj.stop();
    trajectory_active_ = false;
    return true;
}

const std::string& AdmittanceState::check_switch() const {
    if (factory != nullptr && !trajectory_active_ && !factory->exp_state_name.empty()
        && factory->exp_state_name != fsm_name_) {
        return factory->exp_state_name;
    }
    return fsm_name_;
}

bool AdmittanceState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->arm_solve_ == nullptr || factory->model_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    if (!trajectory_active_) {
        write_hold_command(factory, joint_count_, hold_position_, default_kp_, default_kd_, &gravity_, fsm_name_.c_str());
        return true;
    }

    traj.update(time, point);
    if (point.pos.size() != static_cast<Eigen::Index>(task_dof_) || point.vel.size() != static_cast<Eigen::Index>(task_dof_)
        || point.acc.size() != static_cast<Eigen::Index>(task_dof_) || !point.pos.allFinite() || !point.vel.allFinite()
        || !point.acc.allFinite()) {
        traj.stop();
        trajectory_active_ = false;
        set_fault_stage("Admittance trajectory sampling");
        return false;
    }

    double dt         = (time - last_update_time_).seconds();
    last_update_time_ = time;
    if (!std::isfinite(dt) || dt < 0.0) {
        dt = 0.0;
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        const auto index       = static_cast<Eigen::Index>(i);
        joint_pos_(index)      = factory->state_[i].position;
        joint_velocity_(index) = factory->state_[i].velocity;
    }
    joint_acceleration_.setZero();

    try {
        if (!factory->model_->inverse_dynamic(joint_pos_, joint_velocity_, joint_acceleration_, &model_torque_)) {
            traj.stop();
            trajectory_active_ = false;
            set_fault_stage("Admittance inverse dynamics");
            return false;
        }
    } catch (const std::exception&) {
        traj.stop();
        trajectory_active_ = false;
        set_fault_stage("Admittance inverse dynamics");
        return false;
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        torque_residual_(static_cast<Eigen::Index>(i)) = factory->state_[i].torque - model_torque_(static_cast<Eigen::Index>(i));
    }
    try {
        if (!factory->arm_solve_->static_force(joint_pos_, torque_residual_, &task_force_)) {
            traj.stop();
            trajectory_active_ = false;
            set_fault_stage("Admittance static force mapping");
            return false;
        }
    } catch (const std::exception&) {
        traj.stop();
        trajectory_active_ = false;
        set_fault_stage("Admittance static force mapping");
        return false;
    }

    position_error_.noalias() = desired_task_position_ - point.pos;
    velocity_error_.noalias() = desired_task_velocity_ - point.vel;
    desired_task_acceleration_ =
        point.acc
        + (task_force_ - admittance_damping_.cwiseProduct(velocity_error_) - admittance_stiffness_.cwiseProduct(position_error_))
              .cwiseQuotient(admittance_mass_);
    desired_task_velocity_.noalias() += desired_task_acceleration_ * dt;
    desired_task_position_.noalias() += desired_task_velocity_ * dt;
    if (!desired_task_position_.allFinite() || !desired_task_velocity_.allFinite() || !desired_task_acceleration_.allFinite()) {
        traj.stop();
        trajectory_active_ = false;
        set_fault_stage("Admittance command integration");
        return false;
    }

    try {
        task_force_.setZero();
        if (!factory->arm_solve_->inverse_kinamic(desired_task_position_, &joint_pos_)
            || !factory->arm_solve_->inverse_velocity(joint_pos_, desired_task_velocity_, &torque_)
            || !factory->arm_solve_->inverse_dynamic(
                joint_pos_, desired_task_velocity_, desired_task_acceleration_, task_force_, &model_torque_)) {
            traj.stop();
            trajectory_active_ = false;
            set_fault_stage("Admittance solve");
            return false;
        }
    } catch (const std::exception&) {
        traj.stop();
        trajectory_active_ = false;
        set_fault_stage("Admittance solve");
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        const auto index = static_cast<Eigen::Index>(i);
        auto& command    = factory->command_[i];
        command.position = static_cast<float>(joint_pos_(index));
        command.velocity = static_cast<float>(torque_(index));
        command.torque   = static_cast<float>(model_torque_(index));
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    if (trajectory_finished(traj, traj_start_time_, time)) {
        traj.stop();
        trajectory_active_ = false;
    }
    return true;
}

TeachPendantState::TeachPendantState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx)) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("default_kd", default_kd_param_);

    const std::size_t param_capacity = std::max(joint_count_, default_kd_param_.size());
    default_kd_param_.reserve(param_capacity);
    default_kd_.resize(joint_count_);
    joint_pos_.resize(joint_count_);
    task_zero_.setZero(6);
    gravity_torque_.resize(joint_count_);
    end_effector_pose_.resize(6);
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
}

bool TeachPendantState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    return true;
}

bool TeachPendantState::exit(const std::string& next_state) {
    return next_state == "idel";
}

const std::string& TeachPendantState::check_switch() const {
    if (factory != nullptr && factory->exp_state_name == kIdelStateName) {
        return kIdelStateName;
    }
    return fsm_name_;
}

bool TeachPendantState::run(const rclcpp::Time& time) {
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->arm_solve_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_dynamic(joint_pos_, task_zero_, task_zero_, task_zero_, &gravity_torque_)
            || !factory->arm_solve_->forward_kinamic(joint_pos_, &end_effector_pose_)) {
            RCLCPP_WARN_THROTTLE(factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Teach pendant solve failed");
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Teach pendant gravity compensation failed: %s",
            error.what());
        return false;
    }

    if (gravity_torque_.size() != static_cast<Eigen::Index>(joint_count_) || !gravity_torque_.allFinite() || end_effector_pose_.size() != 6
        || !end_effector_pose_.allFinite()) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000,
            "Teach pendant solve returned invalid result: gravity torque size=%ld, pose size=%ld", gravity_torque_.size(),
            end_effector_pose_.size());
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        auto& command    = factory->command_[i];
        command.position = factory->state_[i].position;
        command.velocity = 0.0f;
        command.torque   = static_cast<float>(gravity_torque_(static_cast<Eigen::Index>(i)));
        command.kp       = 0.0f;
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    RCLCPP_INFO_THROTTLE(
        factory->node_->get_logger(), *factory->node_->get_clock(), 200,
        "Teach pendant end-effector pose: position[x=%.6f, y=%.6f, z=%.6f], "
        "rotation_vector[rx=%.6f, ry=%.6f, rz=%.6f]",
        end_effector_pose_(0), end_effector_pose_(1), end_effector_pose_(2), end_effector_pose_(3), end_effector_pose_(4),
        end_effector_pose_(5));

    return true;
}



ParamterMeasureState::ParamterMeasureState(const std::string& name, std::any ctx)
    : FSM(name, ctx)
    , factory(std::any_cast<FSMArmControlFactory*>(ctx))
    , move_to_start_trajectory_(factory->model_->dof(), 4) {
    if (factory == nullptr || factory->node_ == nullptr) {
        return;
    }

    std::vector<std::string> joints;
    factory->node_->get_parameter("joints", joints);
    joint_count_ = joints.size();
    factory->node_->get_parameter("default_kp", default_kp_param_);
    factory->node_->get_parameter("default_kd", default_kd_param_);
    factory->node_->get_parameter("measure_trajectory_period", trajectory_period_);
    factory->node_->get_parameter("measure_trajectory_repeat_cnt", trajectory_repeat_cnt_);
    factory->node_->get_parameter("measure_move_to_start_duration", move_to_start_duration_);
    factory->node_->get_parameter("measure_csv_file_path", measure_csv_file_path_);
    factory->node_->get_parameter("measure_record_sample_rate", measure_record_sample_rate_);
    // O10：激励轨迹的随机种子。0 = 每次按当前时间随机（不可复现），非 0 = 固定可复现。
    int64_t measure_trajectory_seed = 0;
    factory->node_->get_parameter("measure_trajectory_seed", measure_trajectory_seed);
    double end_effector_x_lower_limit;
    double end_effector_x_upper_limit;
    double end_effector_y_lower_limit;
    double end_effector_y_upper_limit;
    double end_effector_z_lower_limit;
    double end_effector_z_upper_limit;
    double traj_torque_limit_rate;
    double end_effector_tool_offset;
    double traj_collision_safe_distance;
    double traj_ground_clearance;
    factory->node_->get_parameter("measure_traj_torque_limit_rate", traj_torque_limit_rate);
    factory->node_->get_parameter("measure_end_effector_tool_offset", end_effector_tool_offset);
    // O12：碰撞限制。0 = 关闭。实际生效值会打进 "激励约束：" 日志行。
    factory->node_->get_parameter("measure_traj_collision_safe_distance", traj_collision_safe_distance);
    factory->node_->get_parameter("measure_traj_ground_clearance", traj_ground_clearance);
    factory->node_->get_parameter("measure_end_effector_x_lower_limit", end_effector_x_lower_limit);
    factory->node_->get_parameter("measure_end_effector_x_upper_limit", end_effector_x_upper_limit);
    factory->node_->get_parameter("measure_end_effector_y_lower_limit", end_effector_y_lower_limit);
    factory->node_->get_parameter("measure_end_effector_y_upper_limit", end_effector_y_upper_limit);
    factory->node_->get_parameter("measure_end_effector_z_lower_limit", end_effector_z_lower_limit);
    factory->node_->get_parameter("measure_end_effector_z_upper_limit", end_effector_z_upper_limit);

    const std::size_t param_capacity = std::max({joint_count_, default_kp_param_.size(), default_kd_param_.size()});
    default_kp_param_.reserve(param_capacity);
    default_kd_param_.reserve(param_capacity);
    hold_position_.resize(joint_count_);
    default_kp_.resize(joint_count_);
    default_kd_.resize(joint_count_);
    target_position_.resize(joint_count_);
    excitation_start_position_.resize(joint_count_);
    excitation_end_position_.resize(joint_count_);
    measured_position_.resize(joint_count_);
    measured_velocity_.resize(joint_count_);
    measured_acceleration_.resize(joint_count_);
    measured_torque_.resize(joint_count_);
    prev_position_.resize(joint_count_);
    prev_velocity_.resize(joint_count_);
    start_point_.pos.resize(joint_count_);
    start_point_.vel.resize(joint_count_);
    start_point_.acc.resize(joint_count_);
    end_point_.pos.resize(joint_count_);
    end_point_.vel.resize(joint_count_);
    end_point_.acc.resize(joint_count_);

    trajectory_period_          = trajectory_period_ > 0.0 ? trajectory_period_ : 10.0;
    trajectory_repeat_cnt_      = trajectory_repeat_cnt_ > 0 ? trajectory_repeat_cnt_ : 1;
    move_to_start_duration_     = move_to_start_duration_ > 0.0 ? move_to_start_duration_ : 3.0;
    measure_record_sample_rate_ = measure_record_sample_rate_ > 0.0 ? measure_record_sample_rate_ : 500.0;
    if (measure_csv_file_path_.empty()) {
        measure_csv_file_path_ = "measured_for_identification.csv";
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    const std::string urdf_path                        = factory->node_->get_parameter("urdf_path").as_string();
    excitation_trajectory_                             = std::make_shared<ExcitationTrajectory>(urdf_path);
    excitation_trajectory_->set_seed(static_cast<std::uint64_t>(measure_trajectory_seed));
    excitation_trajectory_->end_effector_x_lower_limit = end_effector_x_lower_limit;
    excitation_trajectory_->end_effector_x_upper_limit = end_effector_x_upper_limit;
    excitation_trajectory_->end_effector_y_lower_limit = end_effector_y_lower_limit;
    excitation_trajectory_->end_effector_y_upper_limit = end_effector_y_upper_limit;
    excitation_trajectory_->end_effector_z_lower_limit = end_effector_z_lower_limit;
    excitation_trajectory_->end_effector_z_upper_limit = end_effector_z_upper_limit;
    excitation_trajectory_->traj_torque_limit_rate    = traj_torque_limit_rate;
    excitation_trajectory_->end_effector_tool_offset  = end_effector_tool_offset;
    excitation_trajectory_->traj_collision_safe_distance = traj_collision_safe_distance;
    excitation_trajectory_->traj_ground_clearance        = traj_ground_clearance;
    paramter_identify_                                 = std::make_unique<ParamterIdentify>(urdf_path);
    const auto record_capacity =
        static_cast<int>(std::ceil(trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_) * measure_record_sample_rate_)) + 2;
    paramter_identify_->application_memory(record_capacity, static_cast<int>(joint_count_));
}

bool ParamterMeasureState::enter(const std::string& last_state, const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || last_state != "idel" || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_ || excitation_trajectory_ == nullptr || paramter_identify_ == nullptr) {
        return false;
    }

    if (!paramter_identify_->reset_recording()) {
        RCLCPP_WARN(factory->node_->get_logger(), "Previous parameter identification CSV save is still running");
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
        command.position  = hold_position_[i];
        command.velocity  = 0.0f;
        command.torque    = 0.0f;
        command.kp        = default_kp_[i];
        command.kd        = default_kd_[i];
        command.ki        = 0.0f;
    }

    measure_done_                 = false;
    trajectory_generation_failed_ = false;
    csv_save_requested_           = false;
    record_full_warned_           = false;
    has_prev_sample_              = false;
    measure_phase_                = MeasurePhase::WaitingForTrajectory;
    phase_start_time_             = time;
    measure_start_time_           = time;
    excitation_trajectory_->generate(trajectory_period_, trajectory_repeat_cnt_);

    return true;
}

bool ParamterMeasureState::exit(const std::string& next_state) {
    return next_state == "idel";
}

const std::string& ParamterMeasureState::check_switch() const {
    if (factory != nullptr && measure_phase_ == MeasurePhase::HoldingEnd && factory->exp_state_name == kIdelStateName) {
        return kIdelStateName;
    }
    return fsm_name_;
}

bool ParamterMeasureState::run(const rclcpp::Time& time) {
    if (factory == nullptr || factory->node_ == nullptr || excitation_trajectory_ == nullptr || paramter_identify_ == nullptr
        || factory->state_.size() != joint_count_ || factory->command_.size() != joint_count_) {
        return false;
    }

    if (measure_phase_ == MeasurePhase::WaitingForTrajectory) {
        for (std::size_t i = 0; i < joint_count_; ++i) {
            auto& command    = factory->command_[i];
            command.position = hold_position_[i];
            command.velocity = 0.0f;
            command.torque   = 0.0f;
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if (trajectory_generation_failed_ || excitation_trajectory_->generate_failed()) {
            return false;
        }
        if (!excitation_trajectory_->generate_is_finished()) {
            return true;
        }

        const auto zero_duration    = rclcpp::Duration::from_seconds(0.0);
        const double total_duration = trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_);
        if (!excitation_trajectory_->get_target_position(zero_duration, excitation_start_position_)
            || !excitation_trajectory_->get_target_position(rclcpp::Duration::from_seconds(total_duration), excitation_end_position_)
            || excitation_start_position_.size() != static_cast<Eigen::Index>(joint_count_)
            || excitation_end_position_.size() != static_cast<Eigen::Index>(joint_count_) || !excitation_start_position_.allFinite()
            || !excitation_end_position_.allFinite()) {
            return false;
        }

        start_point_.time = zero_duration;
        end_point_.time   = zero_duration;

        for (std::size_t i = 0; i < joint_count_; ++i) {
            start_point_.pos(static_cast<Eigen::Index>(i)) = hold_position_[i];
        }
        start_point_.vel.setZero();
        start_point_.acc.setZero();

        end_point_.pos = excitation_start_position_;
        end_point_.vel.setZero();
        end_point_.acc.setZero();

        move_to_start_trajectory_.add_point(start_point_, zero_duration);
        move_to_start_trajectory_.add_point(end_point_, rclcpp::Duration::from_seconds(move_to_start_duration_));
        move_to_start_trajectory_.start(time);
        phase_start_time_ = time;
        // O10：把实际生效的种子与可行性诊断一并打出来——种子用于复现，
        // violation 必须为 0（不为 0 时 generate_failed() 已置位，run() 会直接失败）。
        RCLCPP_INFO(
            factory->node_->get_logger(),
            "激励轨迹就绪：可辨识参数 %d 个，seed=%lu，原始违例 %.3e rad，缩放 %.4f，最终违例 %.3e rad，得分 %.4f",
            excitation_trajectory_->get_available_param_num(),
            static_cast<unsigned long>(excitation_trajectory_->get_seed()),
            excitation_trajectory_->get_raw_violation(),
            excitation_trajectory_->get_applied_scale(),
            excitation_trajectory_->get_final_violation(),
            excitation_trajectory_->get_final_score());
        // 约束的实际生效值也必须打出来（C6：写错的参数名不会有任何提示）。
        // O12：碰撞约束的实际生效值与违例量也在这里打出来——判断「碰撞约束到底有没有在
        // 约束优化」只能看这一行，不能只看 YAML（C6 的教训）。
        RCLCPP_INFO(
            factory->node_->get_logger(),
            "激励约束：关节软限位 %.3f，力矩上限 %.3f×额定，末端工具偏移 %.4f m，末端 z 下限 %.4f m，"
            "连杆间安全间距 %.4f m，地面净空 %.4f m，碰撞违例 %.3e",
            excitation_trajectory_->traj_pos_soft_limit_rate,
            excitation_trajectory_->traj_torque_limit_rate,
            excitation_trajectory_->end_effector_tool_offset,
            excitation_trajectory_->end_effector_z_lower_limit,
            excitation_trajectory_->traj_collision_safe_distance,
            excitation_trajectory_->traj_ground_clearance,
            excitation_trajectory_->get_collision_violation());
        measure_phase_ = MeasurePhase::MovingToStart;
    }

    if (measure_phase_ == MeasurePhase::MovingToStart) {
        move_to_start_trajectory_.update(time, move_to_start_point_);
        if (move_to_start_point_.pos.size() != static_cast<Eigen::Index>(joint_count_) || !move_to_start_point_.pos.allFinite()) {
            return false;
        }

        const bool has_velocity =
            move_to_start_point_.vel.size() == static_cast<Eigen::Index>(joint_count_) && move_to_start_point_.vel.allFinite();
        for (std::size_t i = 0; i < joint_count_; ++i) {
            auto& command    = factory->command_[i];
            command.position = static_cast<float>(move_to_start_point_.pos(static_cast<Eigen::Index>(i)));
            command.velocity = has_velocity ? static_cast<float>(move_to_start_point_.vel(static_cast<Eigen::Index>(i))) : 0.0f;
            command.torque   = 0.0f;
            command.kp       = default_kp_[i];
            command.kd       = default_kd_[i];
            command.ki       = 0.0f;
        }

        if ((time - phase_start_time_).seconds() < move_to_start_duration_) {
            return true;
        }

        measure_start_time_ = time;
        measure_phase_      = MeasurePhase::ExecutingTrajectory;
        return true;
    }

    if (measure_phase_ == MeasurePhase::ExecutingTrajectory) {
        const rclcpp::Duration elapsed = time - measure_start_time_;
        const double total_duration    = trajectory_period_ * static_cast<double>(trajectory_repeat_cnt_);
        const bool trajectory_finished = elapsed.seconds() >= total_duration;
        if (trajectory_finished) {
            target_position_ = excitation_end_position_;
        } else if (
            !excitation_trajectory_->get_target_position(elapsed, target_position_)
            || target_position_.size() != static_cast<Eigen::Index>(joint_count_) || !target_position_.allFinite()) {
            return false;
        }

        const bool has_acceleration = factory->state_acceleration_.size() == joint_count_;
        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index              = static_cast<Eigen::Index>(i);
            measured_position_(index)     = factory->state_[i].position;
            measured_velocity_(index)     = factory->state_[i].velocity;
            measured_torque_(index)       = factory->state_[i].torque;
            // 真实加速度（O8）：MuJoCo 的 qacc。取不到就记 0，报告里会体现出来。
            measured_acceleration_(index) = has_acceleration ? factory->state_acceleration_[i] : 0.0;
        }
        // O8 的最后一环：**时间对齐**。
        //
        // MuJoCo 的 mj_step 是「先 mj_forward 算出 qacc，再积分」：步进结束后
        // mj_data 里的 qacc 是在**步进之前**的状态上算出来的
        //     qacc_k = f(q_{k-1}, v_{k-1}, tau_k)
        // 而 qpos/qvel 已经是步进**之后**的 S_k。两者差一拍，直接配对会让刚体方程
        // M(q)q̈ + h(q,q̇) = τ 对不上——实测模型残差 0.25%（平缓轨迹）。
        // 把位置/速度**延后一拍**再与加速度/力矩配对后，残差降到 **0.0003%**（机器精度）。
        // 时间列同样用上一拍的时间，保持三元组自洽。
        if (has_prev_sample_) {
            const std::size_t remaining = paramter_identify_->record_value(
                prev_position_, prev_velocity_, measured_acceleration_, measured_torque_, prev_time_);
            if (remaining == 0 && !record_full_warned_) {
                // 缓冲区写满：后续样本会被静默丢弃，这里只告警一次，避免实时线程刷屏。
                record_full_warned_ = true;
                RCLCPP_WARN(
                    factory->node_->get_logger(),
                    "measure 记录缓冲区已满（%zu 个样本），后续样本被丢弃；请调大 measure_record_sample_rate",
                    paramter_identify_->record_capacity());
            }
        }
        prev_position_   = measured_position_;
        prev_velocity_   = measured_velocity_;
        prev_time_       = time.seconds();
        has_prev_sample_ = true;

        if (trajectory_finished) {
            measure_done_  = true;
            measure_phase_ = MeasurePhase::HoldingEnd;
            if (!csv_save_requested_) {
                csv_save_requested_ = paramter_identify_->save_csv(measure_csv_file_path_);
            }
        }
    } else if (measure_phase_ == MeasurePhase::HoldingEnd) {
        target_position_ = excitation_end_position_;
    }

    if (target_position_.size() != static_cast<Eigen::Index>(joint_count_) || !target_position_.allFinite()) {
        return false;
    }

    for (std::size_t i = 0; i < joint_count_; ++i) {
        auto& command    = factory->command_[i];
        command.position = static_cast<float>(target_position_(static_cast<Eigen::Index>(i)));
        command.velocity = 0.0f;
        command.torque   = 0.0f;
        command.kp       = default_kp_[i];
        command.kd       = default_kd_[i];
        command.ki       = 0.0f;
    }

    return true;
}
