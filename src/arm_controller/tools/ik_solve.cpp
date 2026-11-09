/**
 * @file ik_solve.cpp
 * @brief 用给定的目标与初值跑一次逆解，用于离线复现仿真中的失败用例
 *
 * 用法：ik_solve [--axis|--position] <urdf> <t0..t5> <s0..s5>
 *   t* = 任务空间目标（位置 + 旋转向量）
 *   s* = 关节角初值
 *   --axis     = 5 维模式（位置 + 工具 z 轴方向，滚转自由），用于验证 pos_move orientation
 *   --position = 3 维模式（只约束位置），用于区分「位置不可达」与「姿态不可达」
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
    // 可选模式 flag：放在最前面，保持旧用法（不带 flag）完全不变。
    int argument_offset = 1;
    bool axis_mode      = false;
    bool position_only  = false;
    if (argc > 1 && std::string(argv[1]) == "--axis") {
        axis_mode       = true;
        argument_offset = 2;
    } else if (argc > 1 && std::string(argv[1]) == "--position") {
        position_only   = true;
        argument_offset = 2;
    }
    if (argc < 13 + argument_offset) {
        fprintf(stderr, "usage: ik_solve [--axis|--position] <urdf> <t0..t5> <s0..s5>\n");
        return 1;
    }

    auto model   = std::make_shared<ModelFromURDF>(argv[argument_offset], "link6");
    auto mapping = std::make_shared<Default6DofTaskSpaceMapping>();
    IKSolver solver(model, mapping);
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    if (axis_mode) {
        mapping->set_pose_mode(PoseMode::kPositionAndToolAxis);
        solver.set_solve_options(1e-3, 1.0 * kDegToRad, 0.005);
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
