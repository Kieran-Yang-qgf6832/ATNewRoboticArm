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

void hold_current_joint_position(
    FSMArmControlFactory* factory, std::size_t joint_count, const std::vector<float>& default_kp, const std::vector<float>& default_kd) {
    if (factory == nullptr || factory->state_.size() != joint_count || factory->command_.size() != joint_count
        || default_kp.size() != joint_count || default_kd.size() != joint_count) {
        return;
    }

    for (std::size_t i = 0; i < joint_count; ++i) {
        auto& command    = factory->command_[i];
        command.position = factory->state_[i].position;
        command.velocity = 0.0f;
        command.torque   = 0.0f;
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
}

bool IDELState::enter(const std::string& last_state, const rclcpp::Time& time) {
    (void)last_state;
    (void)time;

    if (factory == nullptr || factory->node_ == nullptr || factory->state_.size() != joint_count_
        || factory->command_.size() != joint_count_) {
        return false;
    }

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        hold_position_[i] = factory->state_[i].position;
        auto& command     = factory->command_[i];
        command.position  = hold_position_[i];
        command.velocity  = 0.0f;
        command.torque    = 0.0f;
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

    for (std::size_t i = 0; i < hold_position_.size(); ++i) {
        auto& command    = factory->command_[i];
        command.position = hold_position_[i];
        command.velocity = 0.0f;
        command.torque   = 0.0f;
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
    factory->node_->get_parameter("reset_duration", reset_duration_param);
    factory->node_->get_parameter("reset_tolerance", reset_tolerance_param);
    reset_duration_  = reset_duration_param > 0.0 ? static_cast<float>(reset_duration_param) : 1.0f;
    reset_tolerance_ = reset_tolerance_param >= 0.0 ? static_cast<float>(reset_tolerance_param) : 0.01f;
    for (std::size_t i = 0; i < joint_count_; ++i) {
        reset_joint_pos_[i] = i < reset_joint_pos_param_.size() ? static_cast<float>(reset_joint_pos_param_[i]) : 0.0f;
        default_kp_[i]      = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i]      = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }
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
    for (std::size_t i = 0; i < joint_count_; ++i) {
        const float delta          = reset_joint_pos_[i] - start_joint_pos_[i];
        auto& command              = factory->command_[i];
        command.position           = start_joint_pos_[i] + delta * progress_;
        command.velocity           = progress_ < 1.0f ? delta / reset_duration_ : 0.0f;
        command.torque             = 0.0f;
        command.kp                 = default_kp_[i];
        command.kd                 = default_kd_[i];
        command.ki                 = 0.0f;
        const float position_error = std::fabs(factory->state_[i].position - reset_joint_pos_[i]);
        joint_reached              = joint_reached && position_error <= reset_tolerance_;
    }

    reset_done_ = progress_ >= 1.0f && joint_reached;
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

    cart_traj_cmd_sub_ = factory->node_->create_subscription<robot_msgs::msg::CartTrajCmd>(
        "arm_cart_traj", rclcpp::QoS(10), [this](const robot_msgs::msg::CartTrajCmd& msg) {
            if (state != TrajPhase::STOP || !load_trajectory_from_message(msg.position, msg.seconds, task_dof_, &traj)) {
                return;
            }
            const auto now = factory->node_->get_clock()->now();
            traj.start(now);
            traj_start_time_ = now;
            state            = TrajPhase::MOVING;
        });
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
    hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
    return true;
}

bool CartTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    return true;
}

const std::string& CartTrajState::check_switch() const {
    if (factory != nullptr && state == TrajPhase::STOP && !factory->exp_state_name.empty()
        && factory->exp_state_name != fsm_name_) {
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
        hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
        return true;
    }

    traj.update(time, point);
    if (point.pos.size() != static_cast<Eigen::Index>(task_dof_) || point.vel.size() != static_cast<Eigen::Index>(task_dof_)
        || point.acc.size() != static_cast<Eigen::Index>(task_dof_) || !point.pos.allFinite() || !point.vel.allFinite()
        || !point.acc.allFinite()) {
        return false;
    }

    // 用当前实测关节角作为逆解初值：轨迹首点就是当前位姿，从这个种子出发误差近似为 0，
    // 后续点也是局部小范围变化，避免从零位形起步导致的迭代不收敛。
    for (std::size_t i = 0; i < joint_count_; ++i) {
        joint_pos_(static_cast<Eigen::Index>(i)) = factory->state_[i].position;
    }

    try {
        if (!factory->arm_solve_->inverse_kinamic(point.pos, &joint_pos_)
            || joint_pos_.size() != static_cast<Eigen::Index>(joint_count_) || !joint_pos_.allFinite()
            || !factory->arm_solve_->inverse_dynamic(joint_pos_, point.vel, point.acc, task_force_, &torque_)
            || torque_.size() != static_cast<Eigen::Index>(joint_count_) || !torque_.allFinite()) {
            return false;
        }
    } catch (const std::exception& error) {
        RCLCPP_WARN_THROTTLE(
            factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Cartesian trajectory solve failed: %s", error.what());
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
    hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
    return true;
}

bool JointTrajState::exit(const std::string& next_state) {
    (void)next_state;
    traj.stop();
    state = TrajPhase::STOP;
    return true;
}

const std::string& JointTrajState::check_switch() const {
    if (state == TrajPhase::STOP && !factory->exp_state_name.empty() && factory->exp_state_name != fsm_name_) {
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
        hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
        return true;
    } else if (state == TrajPhase::MOVING) {
        traj.update(time, point);
        if (point.pos.size() != static_cast<Eigen::Index>(joint_count_) || point.vel.size() != static_cast<Eigen::Index>(joint_count_)
            || point.acc.size() != static_cast<Eigen::Index>(joint_count_) || !point.pos.allFinite() || !point.vel.allFinite()
            || !point.acc.allFinite()) {
            return false;
        }

        try {
            if (!factory->model_->inverse_dynamic(point.pos, point.vel, point.acc, &torque)
                || torque.size() != static_cast<Eigen::Index>(joint_count_) || !torque.allFinite()) {
                return false;
            }
        } catch (const std::exception& error) {
            RCLCPP_WARN_THROTTLE(
                factory->node_->get_logger(), *factory->node_->get_clock(), 1000, "Joint trajectory inverse dynamics failed: %s",
                error.what());
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
            return false;
        }
    } catch (const std::exception&) {
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
    return true;
}

bool AdmittanceState::exit(const std::string& next_state) {
    (void)next_state;
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
        hold_current_joint_position(factory, joint_count_, default_kp_, default_kd_);
        return true;
    }

    traj.update(time, point);
    if (point.pos.size() != static_cast<Eigen::Index>(task_dof_) || point.vel.size() != static_cast<Eigen::Index>(task_dof_)
        || point.acc.size() != static_cast<Eigen::Index>(task_dof_) || !point.pos.allFinite() || !point.vel.allFinite()
        || !point.acc.allFinite()) {
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
            return false;
        }
    } catch (const std::exception&) {
        return false;
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        torque_residual_(static_cast<Eigen::Index>(i)) = factory->state_[i].torque - model_torque_(static_cast<Eigen::Index>(i));
    }
    try {
        if (!factory->arm_solve_->static_force(joint_pos_, torque_residual_, &task_force_)) {
            return false;
        }
    } catch (const std::exception&) {
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
        return false;
    }

    try {
        task_force_.setZero();
        if (!factory->arm_solve_->inverse_kinamic(desired_task_position_, &joint_pos_)
            || !factory->arm_solve_->inverse_velocity(joint_pos_, desired_task_velocity_, &torque_)
            || !factory->arm_solve_->inverse_dynamic(
                joint_pos_, desired_task_velocity_, desired_task_acceleration_, task_force_, &model_torque_)) {
            return false;
        }
    } catch (const std::exception&) {
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
    double end_effector_x_lower_limit;
    double end_effector_x_upper_limit;
    double end_effector_y_lower_limit;
    double end_effector_y_upper_limit;
    double end_effector_z_lower_limit;
    double end_effector_z_upper_limit;
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
    measured_torque_.resize(joint_count_);
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
        measure_csv_file_path_ = "/tmp/measured_for_identification.csv";
    }
    for (std::size_t i = 0; i < joint_count_; ++i) {
        default_kp_[i] = i < default_kp_param_.size() ? static_cast<float>(default_kp_param_[i]) : 0.0f;
        default_kd_[i] = i < default_kd_param_.size() ? static_cast<float>(default_kd_param_[i]) : 0.0f;
    }

    const std::string urdf_path                        = factory->node_->get_parameter("urdf_path").as_string();
    excitation_trajectory_                             = std::make_shared<ExcitationTrajectory>(urdf_path);
    excitation_trajectory_->end_effector_x_lower_limit = end_effector_x_lower_limit;
    excitation_trajectory_->end_effector_x_upper_limit = end_effector_x_upper_limit;
    excitation_trajectory_->end_effector_y_lower_limit = end_effector_y_lower_limit;
    excitation_trajectory_->end_effector_y_upper_limit = end_effector_y_upper_limit;
    excitation_trajectory_->end_effector_z_lower_limit = end_effector_z_lower_limit;
    excitation_trajectory_->end_effector_z_upper_limit = end_effector_z_upper_limit;
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
        RCLCPP_INFO(factory->node_->get_logger(), "最小可辨识参数数量为:%d", excitation_trajectory_->get_available_param_num());
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

        for (std::size_t i = 0; i < joint_count_; ++i) {
            const auto index          = static_cast<Eigen::Index>(i);
            measured_position_(index) = factory->state_[i].position;
            measured_velocity_(index) = factory->state_[i].velocity;
            measured_torque_(index)   = factory->state_[i].torque;
        }
        paramter_identify_->record_value(measured_position_, measured_velocity_, measured_torque_);

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
