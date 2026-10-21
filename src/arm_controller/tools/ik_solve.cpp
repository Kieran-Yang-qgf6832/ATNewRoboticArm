/**
 * @file ik_solve.cpp
 * @brief 用给定的目标与初值跑一次逆解，用于离线复现仿真中的失败用例
 *
 * 用法：
 * @code{.sh}
 * ik_solve <urdf> <frame> <task_dof> <t0..t{N-1}> <s0..s{M-1}>
 * @endcode
 *   frame    = 末端坐标系名（4 轴模型 link4，6 轴模型 link6）
 *   task_dof = 任务空间维度（4 轴模型 4，6 轴模型 6）
 *   t*       = 任务空间目标：4 维是 [x, y, z, q4]；6 维是 [x, y, z, rx, ry, rz]
 *   s*       = 关节角初值（个数必须等于模型关节数）
 *
 * @note 只用于离线排查，不参与控制器运行。
 */

#include "default4dof_task.hpp"
#include "default6dof_task.hpp"
#include "kinamic.hpp"
#include "model_from_urdf.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include <Eigen/Dense>

int main(int argc, char** argv) {
    if (argc < 5) {
        fprintf(stderr, "usage: ik_solve <urdf> <frame> <task_dof> <t0..t{N-1}> <s0..s{M-1}>\n");
        return 1;
    }

    const std::string urdf_path = argv[1];
    const std::string frame     = argv[2];
    const int task_dof          = std::atoi(argv[3]);

    std::shared_ptr<ModelBase> model;
    try {
        model = std::make_shared<ModelFromURDF>(urdf_path, frame);
    } catch (const std::exception& error) {
        fprintf(stderr, "failed to load model: %s\n", error.what());
        return 1;
    }

    const int dof = model->dof();
    if (argc != 4 + task_dof + dof) {
        fprintf(
            stderr, "expected %d task values and %d joint values, got %d arguments in total\n", task_dof, dof, argc - 1);
        return 1;
    }

    std::shared_ptr<TaskMapping> mapping;
    if (task_dof == 4) {
        mapping = std::make_shared<Default4DofTaskSpaceMapping>();
    } else if (task_dof == 6) {
        mapping = std::make_shared<Default6DofTaskSpaceMapping>();
    } else {
        fprintf(stderr, "task_dof must be 4 or 6\n");
        return 1;
    }

    IKSolver solver(model, mapping);

    Eigen::VectorXd target(task_dof);
    Eigen::VectorXd seed(dof);
    for (int i = 0; i < task_dof; ++i) {
        target(i) = std::atof(argv[4 + i]);
    }
    for (int i = 0; i < dof; ++i) {
        seed(i) = std::atof(argv[4 + task_dof + i]);
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
    const bool converged     = result == IKSolver::SolveResult::Converged;

    const char* result_name = "Failed (numerical, reachability unknown)";
    if (result == IKSolver::SolveResult::Converged) {
        result_name = "Converged";
    } else if (result == IKSolver::SolveResult::Unreachable) {
        result_name = "Unreachable (target not reached; residual = distance to the nearest reachable pose)";
    }

    printf("converged  = %s\n", converged ? "yes" : "NO");
    printf("result     = %s\n", result_name);
    printf("ik_err     = %.6e (pos %.6e m / rot %.6e rad)\n", solver.last_error_norm(),
           solver.last_position_error_norm(), solver.last_rotation_error_norm());
    printf("iterations = %d\n", solver.last_iterations());
    printf("solve_time = %.1f us/次（%d 次平均）\n", microseconds_per_solve, kRepeatCount);
    printf("solution   = [");
    for (int i = 0; i < dof; ++i) {
        printf("%s%.9f", i == 0 ? "" : ", ", solution(i));
    }
    printf("]\n");

    Eigen::VectorXd achieved(task_dof);
    if (mapping->position_map(solution, model->forward_kinematics(solution), &achieved)) {
        printf("fk(sol)    = [");
        for (int i = 0; i < task_dof; ++i) {
            printf("%s%.9f", i == 0 ? "" : ", ", achieved(i));
        }
        printf("]\n");
    }
    return 0;
}
