/**
 * @file waypoint_planner_test.cpp
 * @brief WaypointTrajectory 的离线自检：节点插值、C3 连续性、峰值限位与总时长语义。
 *
 * 用例覆盖「用户路点 1..8 个」：第一项自动补一个合成起点，因此节点数从 2 到 9，
 * 正好用满 configure(6, 9) 的 max_nodes。路点在关节空间沿平缓直线分布，首末差明显
 * 但远离关节限位，避免位置过冲校验被误触发。
 *
 * 另外包含一组回归用例：tasks.yaml 的 waypoints_joint_8 真实路点 + reset 容差内
 * 的多个起点，以及重合路点（零弦长段）场景（docs/bugs.md O16）。
 *
 * 用法（工作区根目录）：
 * @code{.sh}
 * ./build/arm_controller/waypoint_planner_test
 * @endcode
 *
 * @note 只用于离线校验，不参与控制器运行；任何一项失败返回退出码 1。
 */

#include "waypoint_trajectory.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace {

constexpr int kDof          = 6;
constexpr int kMaxNodes     = 9;
constexpr int kMaxUserPoint = 8;

/// 峰值与限位比较时的数值余量。
constexpr double kLimitMargin = 1e-6;
/// 节点位置插值精度。
constexpr double kPositionTolerance = 1e-9;
/// 节点两侧导数一致性精度（数值差分本身有误差）。
constexpr double kDerivativeTolerance = 1e-4;
/// 探测节点左右导数时的偏移。
constexpr double kProbeOffset = 1e-6;
/// jerk 在节点处只是 C0（四阶导会跳变），±1e-6 的探测本身带 ~1e-4 量级真实变化，
/// 因此探测 jerk 时改用更小的偏移。
constexpr double kJerkProbeOffset = 1e-7;
/// 每段限位检查的采样点数（与 plan 内部的 128 点互为子集）。
constexpr int kLimitSamples = 64;
/// 端点速度/加速度/jerk 应严格为 0（容许 Bezier 控制点公式的浮点抵消误差）。
constexpr double kZeroTolerance = 1e-8;
/// 采样法检查“经过路点”的容差：4000 个采样点之间的间隔本身约 1e-4 量级。
constexpr double kReachTolerance = 1e-3;

int g_failures = 0;

void expect(bool ok, const std::string& what) {
    if (!ok) {
        printf("    FAIL: %s\n", what.c_str());
        ++g_failures;
    }
}

double max_abs(const Eigen::VectorXd& value) {
    return value.size() == 0 ? 0.0 : value.cwiseAbs().maxCoeff();
}

/// 检查整条轨迹是否超限：每段均匀采样 kLimitSamples 个点。
void check_limits(const WaypointTrajectory& trajectory, const WaypointTrajectory::Limits& limits) {
    Eigen::VectorXd pos(kDof);
    Eigen::VectorXd vel(kDof);
    Eigen::VectorXd acc(kDof);
    Eigen::VectorXd jerk(kDof);

    for (std::size_t segment = 0; segment + 1 < trajectory.node_count(); ++segment) {
        const double t0 = trajectory.node_time(segment);
        const double t1 = trajectory.node_time(segment + 1);
        for (int sample = 0; sample <= kLimitSamples; ++sample) {
            const double ratio = static_cast<double>(sample) / kLimitSamples;
            const double t     = t0 + (t1 - t0) * ratio;
            if (!trajectory.sample(t, &pos, &vel, &acc, &jerk)) {
                expect(false, "sample 返回 false");
                return;
            }
            for (int j = 0; j < kDof; ++j) {
                const Eigen::Index k = j;
                expect(
                    std::fabs(vel(k)) <= limits.max_vel(k) * (1.0 + kLimitMargin),
                    "段 " + std::to_string(segment) + " 关节 " + std::to_string(j) + " 速度 " + std::to_string(vel(k))
                        + " 超限 " + std::to_string(limits.max_vel(k)));
                expect(
                    std::fabs(acc(k)) <= limits.max_acc(k) * (1.0 + kLimitMargin),
                    "段 " + std::to_string(segment) + " 关节 " + std::to_string(j) + " 加速度 " + std::to_string(acc(k))
                        + " 超限 " + std::to_string(limits.max_acc(k)));
                expect(
                    std::fabs(jerk(k)) <= limits.max_jerk(k) * (1.0 + kLimitMargin),
                    "段 " + std::to_string(segment) + " 关节 " + std::to_string(j) + " jerk " + std::to_string(jerk(k))
                        + " 超限 " + std::to_string(limits.max_jerk(k)));
            }
        }
    }
}

/// 检查路点是否被精确穿过，以及内部节点两侧的 v/a/j 是否连续。
void check_nodes(const WaypointTrajectory& trajectory, const std::vector<Eigen::VectorXd>& nodes) {
    Eigen::VectorXd pos(kDof);
    Eigen::VectorXd vel(kDof);
    Eigen::VectorXd acc(kDof);
    Eigen::VectorXd jerk(kDof);

    for (std::size_t i = 1; i + 1 < nodes.size(); ++i) {
        const double t = trajectory.node_time(i);
        if (!trajectory.sample(t, &pos, &vel, &acc, &jerk)) {
            expect(false, "sample 返回 false");
            return;
        }

        expect(
            (pos - nodes[i]).cwiseAbs().maxCoeff() < kPositionTolerance,
            "节点 " + std::to_string(i) + " 位置误差 " + std::to_string((pos - nodes[i]).cwiseAbs().maxCoeff()));

        // 节点处的导数即「节点值」：与左右两侧（容差内）应一致，验证 C3 连续。
        const Eigen::VectorXd ref_vel  = vel;
        const Eigen::VectorXd ref_acc  = acc;
        const Eigen::VectorXd ref_jerk = jerk;

        for (const double probe : {t - kProbeOffset, t + kProbeOffset}) {
            if (probe < 0.0 || probe > trajectory.duration()) {
                continue;
            }
            if (!trajectory.sample(probe, &pos, &vel, &acc, &jerk)) {
                expect(false, "sample 返回 false");
                return;
            }
            expect(
                (vel - ref_vel).cwiseAbs().maxCoeff() < kDerivativeTolerance,
                "节点 " + std::to_string(i) + " 附近速度不连续，误差 " + std::to_string((vel - ref_vel).cwiseAbs().maxCoeff()));
            expect(
                (acc - ref_acc).cwiseAbs().maxCoeff() < kDerivativeTolerance,
                "节点 " + std::to_string(i) + " 附近加速度不连续，误差 " + std::to_string((acc - ref_acc).cwiseAbs().maxCoeff()));
        }

        for (const double probe : {t - kJerkProbeOffset, t + kJerkProbeOffset}) {
            if (probe < 0.0 || probe > trajectory.duration()) {
                continue;
            }
            if (!trajectory.sample(probe, &pos, &vel, &acc, &jerk)) {
                expect(false, "sample 返回 false");
                return;
            }
            expect(
                (jerk - ref_jerk).cwiseAbs().maxCoeff() < kDerivativeTolerance,
                "节点 " + std::to_string(i) + " 附近 jerk 不连续，误差 " + std::to_string((jerk - ref_jerk).cwiseAbs().maxCoeff()));
        }
    }
}

/// 检查端点静止：t = 0 与 t = duration 处的速度/加速度/jerk 应为 0。
void check_endpoints(const WaypointTrajectory& trajectory) {
    Eigen::VectorXd pos(kDof);
    Eigen::VectorXd vel(kDof);
    Eigen::VectorXd acc(kDof);
    Eigen::VectorXd jerk(kDof);

    for (const double t : {0.0, trajectory.duration()}) {
        if (!trajectory.sample(t, &pos, &vel, &acc, &jerk)) {
            expect(false, "sample 返回 false");
            return;
        }
        expect(max_abs(vel) < kZeroTolerance, "t=" + std::to_string(t) + " 速度不为 0：" + std::to_string(max_abs(vel)));
        expect(max_abs(acc) < kZeroTolerance, "t=" + std::to_string(t) + " 加速度不为 0：" + std::to_string(max_abs(acc)));
        expect(max_abs(jerk) < kZeroTolerance, "t=" + std::to_string(t) + " jerk 不为 0：" + std::to_string(max_abs(jerk)));
    }
}

/// 检查一组期望路点是否都被轨迹精确经过（按最接近的采样点算 max-abs 距离）。
void check_waypoints_reached(const WaypointTrajectory& trajectory, const std::vector<Eigen::VectorXd>& expected) {
    Eigen::VectorXd pos(kDof);
    Eigen::VectorXd vel(kDof);
    Eigen::VectorXd acc(kDof);
    Eigen::VectorXd jerk(kDof);

    constexpr int kSamples = 4000;
    for (std::size_t i = 0; i < expected.size(); ++i) {
        double best = 1e9;
        for (int sample = 0; sample <= kSamples; ++sample) {
            const double t = trajectory.duration() * static_cast<double>(sample) / kSamples;
            if (!trajectory.sample(t, &pos, &vel, &acc, &jerk)) {
                expect(false, "sample 返回 false");
                return;
            }
            best = std::min(best, (pos - expected[i]).cwiseAbs().maxCoeff());
        }
        expect(best < kReachTolerance, "期望路点 " + std::to_string(i) + " 未被经过，最小距离 " + std::to_string(best));
    }
}

/**
 * @brief 回归用例：tasks.yaml 里 waypoints_joint_8 的真实路点 + 不同 reset 残差起点。
 *
 * 旧实现按段独立缩放时长，节点导数用“当时的时间间隔”差分估计，起点在 reset 容差内
 * 轻微变化就可能让某段时长塌到最小值，进而把相邻段的三阶导数放大成巨大的 Bezier
 * 控制点，规划出的曲线过冲撞上 joint1 的 ±1.8 限位（docs/bugs.md O16）。
 */
void run_waypoint_regression(
    const WaypointTrajectory::Limits& limits, const Eigen::VectorXd& lower, const Eigen::VectorXd& upper) {
    std::vector<Eigen::VectorXd> waypoints;
    const double raw[kMaxUserPoint][kDof] = {
        {0.030, 0.015, 0.060, -0.010, 0.075, 0.020},
        {0.060, 0.030, 0.115, -0.020, 0.145, 0.040},
        {0.088, 0.046, 0.165, -0.028, 0.208, 0.055},
        {0.112, 0.062, 0.212, -0.034, 0.265, 0.064},
        {0.132, 0.077, 0.256, -0.038, 0.318, 0.068},
        {0.150, 0.089, 0.300, -0.040, 0.368, 0.066},
        {0.173, 0.097, 0.345, -0.040, 0.418, 0.058},
        {0.200, 0.100, 0.392, -0.038, 0.468, 0.045},
    };
    for (const auto& row : raw) {
        Eigen::VectorXd point(kDof);
        for (int j = 0; j < kDof; ++j) {
            point(j) = row[j];
        }
        waypoints.push_back(point);
    }

    // reset 容差 0.01 rad 内的几种起点（含零位），旧实现在这些起点上会间歇性失败。
    std::vector<Eigen::VectorXd> starts;
    starts.push_back(Eigen::VectorXd::Zero(kDof));
    Eigen::VectorXd perturbed(kDof);
    perturbed << 0.009, -0.008, 0.006, -0.004, 0.003, -0.002;
    starts.push_back(perturbed);
    perturbed << -0.009, 0.007, -0.005, 0.004, -0.003, 0.002;
    starts.push_back(perturbed);
    perturbed << 0.005, 0.010, -0.008, -0.006, 0.004, 0.001;
    starts.push_back(perturbed);

    for (std::size_t case_index = 0; case_index < starts.size(); ++case_index) {
        std::vector<Eigen::VectorXd> nodes;
        nodes.push_back(starts[case_index]);
        nodes.insert(nodes.end(), waypoints.begin(), waypoints.end());

        WaypointTrajectory trajectory;
        trajectory.configure(kDof, kMaxNodes);
        std::string error;
        const bool planned = trajectory.plan(nodes, limits, 0.0, lower, upper, &error);
        expect(planned, "回归起点 " + std::to_string(case_index) + " plan 失败：" + error);
        if (!planned) {
            printf("[FAIL] 回归起点 %zu plan 失败：%s\n", case_index, error.c_str());
            continue;
        }
        expect(trajectory.duration() > 0.0 && trajectory.duration() < 20.0, "回归起点时长不合理");
        check_endpoints(trajectory);
        check_nodes(trajectory, nodes);
        check_limits(trajectory, limits);
        printf("[PASS] 回归起点 %zu duration=%.6f s\n", case_index, trajectory.duration());
    }

    // 首点与当前位姿完全相同时会产生零弦长段：必须合并，而不是除零/过冲。
    {
        std::vector<Eigen::VectorXd> nodes;
        nodes.push_back(waypoints.front());
        nodes.insert(nodes.end(), waypoints.begin(), waypoints.end());
        WaypointTrajectory trajectory;
        trajectory.configure(kDof, kMaxNodes);
        std::string error;
        const bool planned = trajectory.plan(nodes, limits, 0.0, lower, upper, &error);
        expect(planned, "重合首点 plan 失败：" + error);
        if (planned) {
            check_endpoints(trajectory);
            check_limits(trajectory, limits);
            check_waypoints_reached(trajectory, waypoints);
            printf("[PASS] 重合首点 duration=%.6f s\n", trajectory.duration());
        } else {
            printf("[FAIL] 重合首点 plan 失败：%s\n", error.c_str());
        }
    }

    // 所有路点重合：应退化为静止轨迹而不是失败或除零。
    {
        std::vector<Eigen::VectorXd> nodes(3, waypoints.front());
        WaypointTrajectory trajectory;
        trajectory.configure(kDof, kMaxNodes);
        std::string error;
        const bool planned = trajectory.plan(nodes, limits, 0.0, lower, upper, &error);
        expect(planned, "全重合路点 plan 失败：" + error);
        if (planned) {
            check_endpoints(trajectory);
            check_limits(trajectory, limits);
            check_waypoints_reached(trajectory, {waypoints.front()});
            printf("[PASS] 全重合路点 duration=%.6f s\n", trajectory.duration());
        }
    }
}

} // namespace

int main() {
    // 关节限位与 URDF 一致。
    Eigen::VectorXd lower(kDof);
    lower << -1.8, -1.5708, -2.9, -2.0, -0.61087, -3.14159;
    Eigen::VectorXd upper(kDof);
    upper << 1.8, 1.5708, 2.9, 1.8, 0.61087, 3.14159;

    // 平缓的关节空间直线：首末差明显（0.1 ~ 0.6 rad），但远离限位，不会触发过冲。
    const Eigen::VectorXd start = Eigen::VectorXd::Zero(kDof);
    Eigen::VectorXd delta(kDof);
    delta << 0.30, -0.20, 0.25, -0.15, 0.10, 0.60;

    WaypointTrajectory::Limits limits;
    limits.max_vel  = Eigen::VectorXd::Constant(kDof, 1.0);
    limits.max_acc  = Eigen::VectorXd::Constant(kDof, 2.0);
    limits.max_jerk = Eigen::VectorXd::Constant(kDof, 10.0);

    WaypointTrajectory trajectory;
    trajectory.configure(kDof, kMaxNodes);

    for (int user_points = 1; user_points <= kMaxUserPoint; ++user_points) {
        const int failures_before = g_failures;

        // 用户路点 i 与合成起点连线等分：共 user_points + 1 个节点。
        const int node_count = user_points + 1;
        std::vector<Eigen::VectorXd> nodes;
        nodes.reserve(static_cast<std::size_t>(node_count));
        for (int i = 0; i < node_count; ++i) {
            const double ratio = static_cast<double>(i) / static_cast<double>(node_count - 1);
            nodes.push_back(start + ratio * delta);
        }

        std::string error;
        const bool planned = trajectory.plan(nodes, limits, 0.0, lower, upper, &error);
        expect(planned, "自动时长 plan 失败：" + error);
        if (!planned) {
            printf("[FAIL] user_points=%d nodes=%d plan 失败：%s\n", user_points, node_count, error.c_str());
            continue;
        }

        expect(trajectory.duration() > 0.0, "自动时长必须 > 0");
        expect(trajectory.node_count() == nodes.size(), "node_count 与路点数不一致");
        expect(std::fabs(trajectory.node_time(0)) < kZeroTolerance, "首节点时刻不为 0");
        expect(
            std::fabs(trajectory.node_time(nodes.size() - 1) - trajectory.duration()) < 1e-12,
            "末节点时刻不等于总时长");

        check_endpoints(trajectory);
        check_nodes(trajectory, nodes);
        check_limits(trajectory, limits);

        const double auto_duration = trajectory.duration();

        // total_duration 放大：时长必须精确等于用户值。
        const double longer = auto_duration * 1.5;
        error.clear();
        expect(trajectory.plan(nodes, limits, longer, lower, upper, &error), "放大总时长的 plan 失败：" + error);
        expect(
            std::fabs(trajectory.duration() - longer) < 1e-9,
            "放大总时长后 duration=" + std::to_string(trajectory.duration()) + "，期望 " + std::to_string(longer));
        check_endpoints(trajectory);

        // total_duration 小于自动最短时长：必须拒绝。
        error.clear();
        const bool rejected = !trajectory.plan(nodes, limits, auto_duration * 0.5, lower, upper, &error);
        expect(rejected, "总时长小于自动最短时长时应返回 false");
        if (rejected) {
            expect(!error.empty(), "拒绝时应给出 error 说明");
        }

        const bool case_ok = (g_failures == failures_before);
        printf(
            "[%s] user_points=%d nodes=%d auto_duration=%.6f s  duration(1.5x)=%.6f s\n", case_ok ? "PASS" : "FAIL",
            user_points, node_count, auto_duration, longer);
    }

    // 回归：tasks.yaml 真实路点 + reset 容差内的不同起点（O16 的间歇性失败）。
    run_waypoint_regression(limits, lower, upper);

    if (g_failures == 0) {
        printf("summary: all checks passed\n");
        return 0;
    }
    printf("summary: %d check(s) failed\n", g_failures);
    return 1;
}
