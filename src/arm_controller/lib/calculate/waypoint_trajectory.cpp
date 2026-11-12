#include "waypoint_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

/// 归一化弧长差小于该值的相邻路点视为重合，直接合并（避免除零）。
constexpr double kDuplicateEpsilon = 1e-9;
/// 整条轨迹弦长为 0（所有路点重合）时使用的默认总时长（秒）。
constexpr double kStaticDuration = 0.1;
/// 全局时间尺度的最小值，避免零时长。
constexpr double kMinTotalDuration = 1e-3;
/// 时间尺度安全余量：峰值按 1/T、1/T²、1/T³ 解析缩放，留 2% 余量吸收采样误差。
constexpr double kSafetyScale = 1.02;
/// 时间尺度复核轮数上限（正常情况下第一轮就满足）。
constexpr int kMaxScaleRounds = 4;
/// 测量每段速度/加速度/jerk 峰值的采样点数（测试用 64 点，是其超集）。
constexpr int kPeakSamples = 128;
/// 位置过冲校验的每段采样点数。
constexpr int kOvershootSamples = 100;
/// 限位比较的数值余量。
constexpr double kLimitEpsilon = 1e-9;

/// 四个连续节点（start..start+3）上关于归一化弧长的三阶差商，用于估计内部节点的 jerk。
double third_divided_difference(
    const std::vector<Eigen::VectorXd>& values, const std::vector<double>& parameter, std::size_t start, Eigen::Index joint) {
    const double d01 =
        (values[start + 1](joint) - values[start](joint)) / (parameter[start + 1] - parameter[start]);
    const double d12 =
        (values[start + 2](joint) - values[start + 1](joint)) / (parameter[start + 2] - parameter[start + 1]);
    const double d23 =
        (values[start + 3](joint) - values[start + 2](joint)) / (parameter[start + 3] - parameter[start + 2]);
    const double d012 = (d12 - d01) / (parameter[start + 2] - parameter[start]);
    const double d123 = (d23 - d12) / (parameter[start + 3] - parameter[start + 1]);
    return (d123 - d012) / (parameter[start + 3] - parameter[start]);
}

} // namespace

void WaypointTrajectory::configure(int dof, int max_nodes) {
    node_count_ = 0;
    duration_   = 0.0;
    if (dof <= 0 || max_nodes < 2) {
        dof_       = 0;
        max_nodes_ = 0;
        return;
    }

    dof_       = dof;
    max_nodes_ = max_nodes;

    // 全部 buffer 只在这里分配（plan/sample 不再分配）。
    node_pos_.assign(static_cast<std::size_t>(max_nodes), Eigen::VectorXd::Zero(dof));
    node_vel_.assign(static_cast<std::size_t>(max_nodes), Eigen::VectorXd::Zero(dof));
    node_acc_.assign(static_cast<std::size_t>(max_nodes), Eigen::VectorXd::Zero(dof));
    node_jerk_.assign(static_cast<std::size_t>(max_nodes), Eigen::VectorXd::Zero(dof));
    node_u_.assign(static_cast<std::size_t>(max_nodes), 0.0);
    node_time_.assign(static_cast<std::size_t>(max_nodes), 0.0);
    control_.assign(static_cast<std::size_t>(max_nodes - 1) * 8, Eigen::VectorXd::Zero(dof));
    segment_duration_.assign(static_cast<std::size_t>(max_nodes - 1), 0.0);
    segment_start_.assign(static_cast<std::size_t>(max_nodes - 1), 0.0);

    work_pos_ = Eigen::VectorXd::Zero(dof);
    work_vel_ = Eigen::VectorXd::Zero(dof);
    work_acc_ = Eigen::VectorXd::Zero(dof);
    work_jerk_ = Eigen::VectorXd::Zero(dof);
    peak_vel_.assign(static_cast<std::size_t>(dof), 0.0);
    peak_acc_.assign(static_cast<std::size_t>(dof), 0.0);
    peak_jerk_.assign(static_cast<std::size_t>(dof), 0.0);
}

void WaypointTrajectory::rebuild_times() {
    double time    = 0.0;
    node_time_[0]  = 0.0;
    for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
        segment_start_[segment] = time;
        time += segment_duration_[segment];
        node_time_[segment + 1] = time;
    }
    duration_ = time;
}

void WaypointTrajectory::estimate_derivatives() {
    // 首尾节点严格静止（对弧长的导数也为 0），保证起停平顺。
    node_vel_[0].setZero();
    node_acc_[0].setZero();
    node_jerk_[0].setZero();
    node_vel_[node_count_ - 1].setZero();
    node_acc_[node_count_ - 1].setZero();
    node_jerk_[node_count_ - 1].setZero();

    for (std::size_t i = 1; i + 1 < node_count_; ++i) {
        const double u_prev = node_u_[i - 1];
        const double u_cur  = node_u_[i];
        const double u_next = node_u_[i + 1];

        // 三阶差商需要四个连续节点；节点不足 4 个时 jerk 置 0。
        const bool has_jerk = node_count_ >= 4;
        std::size_t jerk_start = 0;
        if (has_jerk) {
            // 优先取 u_{i-1}..u_{i+2}；末端的内部节点退化为 u_{i-2}..u_{i+1}。
            jerk_start = (i + 2 < node_count_) ? (i - 1) : (i - 2);
        }

        for (int j = 0; j < dof_; ++j) {
            const Eigen::Index k = j;
            const double p_prev  = node_pos_[i - 1](k);
            const double p_cur   = node_pos_[i](k);
            const double p_next  = node_pos_[i + 1](k);

            // 一阶：中心一阶差商。
            node_vel_[i](k)  = (p_next - p_prev) / (u_next - u_prev);
            // 二阶：二阶差商 × 2!。
            node_acc_[i](k)  = 2.0 * ((p_next - p_cur) / (u_next - u_cur) - (p_cur - p_prev) / (u_cur - u_prev))
                              / (u_next - u_prev);
            // 三阶：三阶差商 × 3!。
            node_jerk_[i](k) = has_jerk ? 6.0 * third_divided_difference(node_pos_, node_u_, jerk_start, k) : 0.0;
        }
    }
}

void WaypointTrajectory::build_controls() {
    for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
        // 该段在归一化弧长上占的比例。Bezier 控制点只由几何决定，
        // 与总时长 T_total 无关（T 只在 sample/evaluate 的求导缩放里出现）。
        const double du        = node_u_[segment + 1] - node_u_[segment];
        const std::size_t base = segment * 8;
        for (int j = 0; j < dof_; ++j) {
            const Eigen::Index k = j;
            const double p0      = node_pos_[segment](k);
            const double p1      = node_pos_[segment + 1](k);
            const double v0      = node_vel_[segment](k);
            const double v1      = node_vel_[segment + 1](k);
            const double a0      = node_acc_[segment](k);
            const double a1      = node_acc_[segment + 1](k);
            const double j0      = node_jerk_[segment](k);
            const double j1      = node_jerk_[segment + 1](k);

            // 七次 Hermite 的等价 Bernstein 控制点（n = 7，匹配两端 p/v/a/j）。
            // v/a/j 是对归一化弧长的导数，因此这里的“时长”用 du。
            const double b0 = p0;
            const double b1 = p0 + du / 7.0 * v0;
            const double b2 = p0 + 2.0 * du / 7.0 * v0 + du * du / 42.0 * a0;
            const double b3 = p0 - 3.0 * b1 + 3.0 * b2 + du * du * du / 210.0 * j0;
            const double b7 = p1;
            const double b6 = p1 - du / 7.0 * v1;
            const double b5 = p1 - 2.0 * du / 7.0 * v1 + du * du / 42.0 * a1;
            const double b4 = b7 - 3.0 * b6 + 3.0 * b5 - du * du * du / 210.0 * j1;

            control_[base + 0](k) = b0;
            control_[base + 1](k) = b1;
            control_[base + 2](k) = b2;
            control_[base + 3](k) = b3;
            control_[base + 4](k) = b4;
            control_[base + 5](k) = b5;
            control_[base + 6](k) = b6;
            control_[base + 7](k) = b7;
        }
    }
}

void WaypointTrajectory::evaluate(
    std::size_t segment, double u, Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* acc,
    Eigen::VectorXd* jerk) const {
    const double w  = 1.0 - u;
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double u6 = u5 * u;
    const double u7 = u6 * u;
    const double w2 = w * w;
    const double w3 = w2 * w;
    const double w4 = w3 * w;
    const double w5 = w4 * w;
    const double w6 = w5 * w;
    const double w7 = w6 * w;

    // 七次 Bernstein 基（位置）。
    const double b[8] = {w7, 7.0 * u * w6, 21.0 * u2 * w5, 35.0 * u3 * w4, 35.0 * u4 * w3, 21.0 * u5 * w2, 7.0 * u6 * w, u7};
    // 对 u 的一阶导：7 × 六次 Bernstein 基（控制点为 ΔB_i）。
    const double d1[7] = {w6, 6.0 * u * w5, 15.0 * u2 * w4, 20.0 * u3 * w3, 15.0 * u4 * w2, 6.0 * u5 * w, u6};
    // 对 u 的二阶导：42 × 五次 Bernstein 基（控制点为 Δ²B_i）。
    const double d2[6] = {w5, 5.0 * u * w4, 10.0 * u2 * w3, 10.0 * u3 * w2, 5.0 * u4 * w, u5};
    // 对 u 的三阶导：210 × 四次 Bernstein 基（控制点为 Δ³B_i）。
    const double d3[5] = {w4, 4.0 * u * w3, 6.0 * u2 * w2, 4.0 * u3 * w, u4};

    const double duration  = segment_duration_[segment];
    const std::size_t base = segment * 8;
    const double inv_t     = 1.0 / duration;
    const double inv_t2    = inv_t * inv_t;
    const double inv_t3    = inv_t2 * inv_t;

    for (int j = 0; j < dof_; ++j) {
        const Eigen::Index k = j;

        double p = 0.0;
        for (int i = 0; i < 8; ++i) {
            p += b[i] * control_[base + i](k);
        }

        double v = 0.0;
        for (int i = 0; i < 7; ++i) {
            v += d1[i] * (control_[base + i + 1](k) - control_[base + i](k));
        }

        double a = 0.0;
        for (int i = 0; i < 6; ++i) {
            a += d2[i] * (control_[base + i + 2](k) - 2.0 * control_[base + i + 1](k) + control_[base + i](k));
        }

        double jk = 0.0;
        for (int i = 0; i < 5; ++i) {
            jk += d3[i] * (control_[base + i + 3](k) - 3.0 * control_[base + i + 2](k) + 3.0 * control_[base + i + 1](k)
                           - control_[base + i](k));
        }

        (*pos)(k)  = p;
        (*vel)(k)  = 7.0 * v * inv_t;
        (*acc)(k)  = 42.0 * a * inv_t2;
        (*jerk)(k) = 210.0 * jk * inv_t3;
    }
}

double WaypointTrajectory::measure_segment_ratio(const Limits& limits, std::size_t segment) {
    std::fill(peak_vel_.begin(), peak_vel_.end(), 0.0);
    std::fill(peak_acc_.begin(), peak_acc_.end(), 0.0);
    std::fill(peak_jerk_.begin(), peak_jerk_.end(), 0.0);

    for (int sample = 0; sample <= kPeakSamples; ++sample) {
        const double u = static_cast<double>(sample) / kPeakSamples;
        evaluate(segment, u, &work_pos_, &work_vel_, &work_acc_, &work_jerk_);
        for (int j = 0; j < dof_; ++j) {
            const Eigen::Index k = j;
            peak_vel_[j]         = std::max(peak_vel_[j], std::fabs(work_vel_(k)));
            peak_acc_[j]         = std::max(peak_acc_[j], std::fabs(work_acc_(k)));
            peak_jerk_[j]        = std::max(peak_jerk_[j], std::fabs(work_jerk_(k)));
        }
    }

    // 归一到「需要把当前时长放大多少倍才能满足限位」的比值。
    double ratio = 0.0;
    for (int j = 0; j < dof_; ++j) {
        ratio = std::max(ratio, peak_vel_[j] / limits.max_vel(j));
        ratio = std::max(ratio, std::sqrt(peak_acc_[j] / limits.max_acc(j)));
        ratio = std::max(ratio, std::cbrt(peak_jerk_[j] / limits.max_jerk(j)));
    }
    return ratio;
}

bool WaypointTrajectory::plan(
    const std::vector<Eigen::VectorXd>& nodes, const Limits& limits, double total_duration,
    const Eigen::VectorXd& lower_limit, const Eigen::VectorXd& upper_limit, std::string* error) {
    node_count_ = 0;
    duration_   = 0.0;

    const auto fail = [error](const std::string& message) {
        if (error != nullptr) {
            *error = message;
        }
        return false;
    };

    if (dof_ <= 0 || max_nodes_ < 2) {
        return fail("WaypointTrajectory 尚未 configure(dof, max_nodes)");
    }
    if (nodes.size() < 2) {
        return fail("路点数量不足：至少需要 2 个节点");
    }
    if (static_cast<int>(nodes.size()) > max_nodes_) {
        return fail("路点数量 " + std::to_string(nodes.size()) + " 超过 max_nodes=" + std::to_string(max_nodes_));
    }
    if (limits.max_vel.size() != dof_ || limits.max_acc.size() != dof_ || limits.max_jerk.size() != dof_) {
        return fail("limits 的 max_vel/max_acc/max_jerk 长度必须等于 dof");
    }
    if (!std::isfinite(total_duration)) {
        return fail("total_duration 必须是有限值");
    }
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].size() != dof_) {
            return fail("第 " + std::to_string(i) + " 个路点长度不是 dof");
        }
        if (!nodes[i].allFinite()) {
            return fail("第 " + std::to_string(i) + " 个路点含非有限值");
        }
    }
    for (int j = 0; j < dof_; ++j) {
        if (!(limits.max_vel(j) > 0.0) || !std::isfinite(limits.max_vel(j)) || !(limits.max_acc(j) > 0.0)
            || !std::isfinite(limits.max_acc(j)) || !(limits.max_jerk(j) > 0.0) || !std::isfinite(limits.max_jerk(j))) {
            return fail("limits 必须全部为有限正数");
        }
    }

    // 合并连续重合路点：首点常与当前实测位姿相同，保留重复点会导致该段弦长为 0、
    // 弧长参数化除零。合并后所有点都重合时退化为一条静止轨迹。
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (node_count_ == 0 || (nodes[i] - node_pos_[node_count_ - 1]).norm() > kDuplicateEpsilon) {
            node_pos_[node_count_] = nodes[i];
            ++node_count_;
        }
    }

    if (node_count_ == 1) {
        // 所有路点重合：构造成两节点、零位移、指定时长（或默认 0.1 s）的静止轨迹。
        node_pos_[1] = node_pos_[0];
        node_count_  = 2;
    }

    // 归一化累计弧长 u ∈ [0, 1]；时间分配按各段 Δu 成比例。
    double total_chord = 0.0;
    for (std::size_t i = 1; i < node_count_; ++i) {
        total_chord += (node_pos_[i] - node_pos_[i - 1]).norm();
    }
    if (total_chord <= kDuplicateEpsilon) {
        node_u_[0] = 0.0;
        node_u_[1] = 0.0;
        segment_duration_[0] = total_duration > 0.0 ? total_duration : kStaticDuration;
        rebuild_times();
        estimate_derivatives();
        build_controls();
        return true;
    }

    node_u_[0] = 0.0;
    for (std::size_t i = 1; i < node_count_; ++i) {
        node_u_[i] = node_u_[i - 1] + (node_pos_[i] - node_pos_[i - 1]).norm() / total_chord;
    }
    // 消除浮点累计误差，保证最后一段到 1。
    node_u_[node_count_ - 1] = 1.0;

    // 节点导数（对归一化弧长）只估算一次；Bezier 控制点与时间尺度无关。
    estimate_derivatives();
    build_controls();

    // 先在 T_total = 1 下测量峰值所需的缩放倍数。速度/加速度/jerk 峰值分别按
    // 1/T、1/T²、1/T³ 缩放，因此这个倍数是解析的，不需要逐段迭代。
    for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
        segment_duration_[segment] = node_u_[segment + 1] - node_u_[segment];
    }
    rebuild_times();

    double scale = 1.0;
    for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
        const double ratio = measure_segment_ratio(limits, segment);
        if (!std::isfinite(ratio)) {
            return fail("轨迹规划出现非有限峰值");
        }
        scale = std::max(scale, ratio);
    }
    scale = std::max(kMinTotalDuration, scale * kSafetyScale);

    // 复核：按解析缩放理应已经满足，留几轮吸收采样误差（只放大、不缩小）。
    bool feasible = false;
    for (int round = 0; round < kMaxScaleRounds; ++round) {
        for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
            segment_duration_[segment] = (node_u_[segment + 1] - node_u_[segment]) * scale;
        }
        rebuild_times();

        double worst = 0.0;
        for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
            const double ratio = measure_segment_ratio(limits, segment);
            if (!std::isfinite(ratio)) {
                return fail("轨迹规划出现非有限峰值");
            }
            worst = std::max(worst, ratio);
        }
        if (worst <= 1.0) {
            feasible = true;
            break;
        }
        scale *= worst * kSafetyScale;
    }
    if (!feasible) {
        return fail("自动时长收敛失败：峰值仍超出限位");
    }

    const double auto_total = duration_;
    if (total_duration > 0.0) {
        if (total_duration < auto_total) {
            char buffer[256];
            snprintf(buffer, sizeof(buffer), "请求总时长 %.6f s 小于自动最短总时长 %.6f s", total_duration, auto_total);
            return fail(buffer);
        }
        const double factor = total_duration / auto_total;
        for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
            segment_duration_[segment] *= factor;
        }
        rebuild_times();
    }

    for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
        if (!std::isfinite(segment_duration_[segment]) || segment_duration_[segment] <= 0.0) {
            return fail("段时长非法（必须有限且 > 0）");
        }
    }

    // 位置过冲校验：控制点按端点导数外扩，可能让中间位置略微越过路点范围。
    if (lower_limit.size() == dof_ && upper_limit.size() == dof_) {
        for (std::size_t segment = 0; segment + 1 < node_count_; ++segment) {
            for (int sample = 0; sample <= kOvershootSamples; ++sample) {
                const double u = static_cast<double>(sample) / kOvershootSamples;
                evaluate(segment, u, &work_pos_, &work_vel_, &work_acc_, &work_jerk_);
                for (int j = 0; j < dof_; ++j) {
                    const Eigen::Index k     = j;
                    const double value       = work_pos_(k);
                    const bool below         = value < lower_limit(k) - kLimitEpsilon;
                    const bool above         = value > upper_limit(k) + kLimitEpsilon;
                    if (below || above) {
                        const double time = segment_start_[segment] + u * segment_duration_[segment];
                        char buffer[320];
                        snprintf(
                            buffer, sizeof(buffer), "位置过冲：t=%.6f s 关节 %d 位置 %.9f 超出限位 [%.9f, %.9f]", time, j,
                            value, lower_limit(k), upper_limit(k));
                        return fail(buffer);
                    }
                }
            }
        }
    }

    return true;
}

bool WaypointTrajectory::sample(
    double t, Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* acc, Eigen::VectorXd* jerk) const {
    if (node_count_ < 2) {
        return false;
    }
    if (pos == nullptr || vel == nullptr || acc == nullptr || jerk == nullptr) {
        return false;
    }
    if (pos->size() != dof_ || vel->size() != dof_ || acc->size() != dof_ || jerk->size() != dof_) {
        return false;
    }
    if (!std::isfinite(t)) {
        return false;
    }

    const double clamped = std::clamp(t, 0.0, duration_);

    // 段起点单调递增，线性扫描即可（段数 <= max_nodes-1，不分配内存）。
    // 注意条件里的 +2：段下标最大只能是 node_count_-2，不能越过最后一段。
    std::size_t segment = 0;
    while (segment + 2 < node_count_ && clamped >= segment_start_[segment] + segment_duration_[segment]) {
        ++segment;
    }

    double u = (clamped - segment_start_[segment]) / segment_duration_[segment];
    u        = std::clamp(u, 0.0, 1.0);

    evaluate(segment, u, pos, vel, acc, jerk);
    return true;
}

double WaypointTrajectory::duration() const {
    return duration_;
}

bool WaypointTrajectory::empty() const {
    return node_count_ < 2;
}

std::size_t WaypointTrajectory::node_count() const {
    return node_count_;
}

double WaypointTrajectory::node_time(std::size_t index) const {
    if (index >= node_count_) {
        return 0.0;
    }
    return node_time_[index];
}

int WaypointTrajectory::dof() const {
    return dof_;
}
