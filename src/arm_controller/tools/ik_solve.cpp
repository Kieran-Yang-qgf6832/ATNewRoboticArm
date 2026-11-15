/**
 * @file ik_solve.cpp
 * @brief 用给定的目标与初值跑一次逆解，用于离线复现仿真中的失败用例
 *
 * 用法：ik_solve [--axis|--position] [--accept-axis <deg>] <urdf> <t0..t5> <s0..s5>
 *   t* = 任务空间目标（位置 + 旋转向量）
 *   s* = 关节角初值
 *   --axis     = 5 维模式（位置 + 工具 z 轴方向，滚转自由），用于验证 pos_move orientation
 *   --position = 3 维模式（只约束位置），用于区分「位置不可达」与「姿态不可达」
 *   --accept-axis = 5 维模式的**通过**容差（度，默认 10，与控制器 yaml 一致）。
 *                传 1 即回到「只有收敛判据」的严格行为，可与默认值做 A/B：
 *                同一目标、同一初值下，严格模式报 Unreachable、放宽模式报 Converged。
 *
 * @note 只用于离线排查，不参与控制器运行。
 */

#include "default6dof_task.hpp"
#include "kinamic.hpp"
#include "model_from_urdf.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <Eigen/Dense>

int main(int argc, char** argv) {
    // 可选 flag：放在最前面，保持旧用法（不带 flag）完全不变。
    int argument_offset     = 1;
    bool axis_mode          = false;
    bool position_only      = false;
    double accept_axis_deg  = 10.0; // 与控制器 pos_move_axis_tolerance_deg 默认值一致
    while (argument_offset < argc) {
        const std::string flag = argv[argument_offset];
        if (flag == "--axis") {
            axis_mode = true;
            ++argument_offset;
        } else if (flag == "--position") {
            position_only = true;
            ++argument_offset;
        } else if (flag == "--accept-axis" && argument_offset + 1 < argc) {
            accept_axis_deg = std::atof(argv[argument_offset + 1]);
            argument_offset += 2;
        } else {
            break;
        }
    }
    if (argc < 13 + argument_offset) {
        fprintf(stderr, "usage: ik_solve [--axis|--position] [--accept-axis <deg>] <urdf> <t0..t5> <s0..s5>\n");
        return 1;
    }

    auto model   = std::make_shared<ModelFromURDF>(argv[argument_offset], "link6");
    auto mapping = std::make_shared<Default6DofTaskSpaceMapping>();
    IKSolver solver(model, mapping);
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    if (axis_mode) {
        // 与控制器 yaml 的默认值一致：收敛 1°、通过 10°。只按收敛判据会报 Unreachable，
        // 看不出「差 6° 但已经能用」，排查放宽区间时容易误判；--accept-axis 1 可回到严格行为。
        mapping->set_pose_mode(PoseMode::kPositionAndToolAxis);
        solver.set_solve_options(1e-3, 1.0 * kDegToRad, 0.005, accept_axis_deg * kDegToRad);
    } else if (position_only) {
        mapping->set_pose_mode(PoseMode::kPositionOnly);
        solver.set_solve_options(1e-3, 0.0, 0.005);
    }

    const int dof = model->dof();
    Eigen::VectorXd target(6);
    Eigen::VectorXd seed(dof);
    for (int i = 0; i < 6; ++i) {
        target(i) = std::atof(argv[1 + argument_offset + i]);
    }
    for (int i = 0; i < dof; ++i) {
        seed(i) = std::atof(argv[7 + argument_offset + i]);
    }

    // 重复求解以便测出单次耗时：预检要在 500 Hz 控制周期里跑，必须知道它有多贵。
    constexpr int kRepeatCount = 200;
    Eigen::VectorXd solution   = seed;
    const auto started_at      = std::chrono::steady_clock::now();
    auto result                = solver.solve(target, solution);
    for (int repeat = 1; repeat < kRepeatCount; ++repeat) {
        solution = seed;
        result   = solver.solve(target, solution);
    }
    const double microseconds_per_solve =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - started_at).count() / kRepeatCount;
    const bool converged = result == IKSolver::SolveResult::Converged;

    const char* result_name = "Failed (numerical, reachability unknown)";
    if (result == IKSolver::SolveResult::Converged) {
        result_name = "Converged";
    } else if (result == IKSolver::SolveResult::Unreachable) {
        result_name = "Unreachable (target not reached; residual = distance to the nearest reachable pose)";
    }

    printf("converged  = %s\n", converged ? "yes" : "NO");
    printf("result     = %s\n", result_name);
    printf("ik_err     = %.6e\n", solver.last_error_norm());
    if (axis_mode) {
        // 分离判据：位置与工具轴各自报，便于区分「位置不可达」与「工具轴对不齐」。
        printf(
            "pos/axis   = %.6e m / %.4f deg (converge <= 1.0 deg, accept <= %.3g deg, relaxed_accept = %s)\n",
            solver.last_position_error_norm(), solver.last_axis_error_norm() / kDegToRad, accept_axis_deg,
            solver.last_used_relaxed_axis() ? "yes" : "no");
    }
    printf("iterations = %d\n", solver.last_iterations());
    printf("solve_time = %.1f us/次（%d 次平均）\n", microseconds_per_solve, kRepeatCount);
    printf("solution   = [");
    for (int i = 0; i < dof; ++i) {
        printf("%s%.9f", i == 0 ? "" : ", ", solution(i));
    }
    printf("]\n");

    Eigen::VectorXd achieved(6);
    if (mapping->position_map(solution, model->forward_kinematics(solution), &achieved)) {
        printf(
            "fk(sol)    = [%.9f, %.9f, %.9f, %.9f, %.9f, %.9f]\n", achieved(0), achieved(1), achieved(2), achieved(3), achieved(4),
            achieved(5));
    }
    return 0;
}
