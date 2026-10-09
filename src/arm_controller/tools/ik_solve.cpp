/**
 * @file ik_solve.cpp
 * @brief 用给定的目标与初值跑一次逆解，用于离线复现仿真中的失败用例
 *
 * 用法：ik_solve <urdf> <t0..t5> <s0..s5>
 *   t* = 任务空间目标（位置 + 旋转向量）
 *   s* = 关节角初值
 *
 * @note 只用于离线排查，不参与控制器运行。
 */

#include "default6dof_task.hpp"
#include "kinamic.hpp"
#include "model_from_urdf.hpp"

#include <cstdio>
#include <cstdlib>
#include <memory>

#include <Eigen/Dense>

int main(int argc, char** argv) {
    if (argc < 14) {
        fprintf(stderr, "usage: ik_solve <urdf> <t0..t5> <s0..s5>\n");
        return 1;
    }

    auto model   = std::make_shared<ModelFromURDF>(argv[1], "link6");
    auto mapping = std::make_shared<Default6DofTaskSpaceMapping>();
    IKSolver solver(model, mapping);

    const int dof = model->dof();
    Eigen::VectorXd target(6);
    Eigen::VectorXd seed(dof);
    for (int i = 0; i < 6; ++i) {
        target(i) = std::atof(argv[2 + i]);
    }
    for (int i = 0; i < dof; ++i) {
        seed(i) = std::atof(argv[8 + i]);
    }

    Eigen::VectorXd solution = seed;
    const bool converged     = solver.solve(target, solution);

    printf("converged  = %s\n", converged ? "yes" : "NO");
    printf("ik_err     = %.6e\n", solver.last_error_norm());
    printf("iterations = %d\n", solver.last_iterations());
    printf("solution   = [");
    for (int i = 0; i < dof; ++i) {
        printf("%s%.9f", i == 0 ? "" : ", ", solution(i));
    }
    printf("]\n");

    Eigen::VectorXd achieved(6);
    if (mapping->position_map(solution, model->forward_kinematics(solution), &achieved)) {
        printf("fk(sol)    = [%.9f, %.9f, %.9f, %.9f, %.9f, %.9f]\n", achieved(0), achieved(1), achieved(2),
               achieved(3), achieved(4), achieved(5));
    }
    return 0;
}
