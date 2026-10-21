/**
 * @file ik_benchmark.cpp
 * @brief 逆解器离线基准：用于扫描信赖域 / LM 参数
 *
 * 做法：在关节限位内采样若干位形，用正运动学算出目标，再让逆解器从给定的
 * 初值去求解。统计成功率、平均迭代次数与最差残余误差。
 *
 * 之所以要离线基准：调参若走「改代码 -> colcon build -> 起 MuJoCo -> 跑任务」
 * 一轮要两三分钟，而这里一次扫描几百个用例只要毫秒级。
 *
 * 用法：
 * @code{.sh}
 * ik_benchmark <urdf> <frame> <task_dof> [用例数] [lambda0] [trust0] [trust_grow] [trust_shrink] [seed_mode]
 * @endcode
 *   frame    = 末端坐标系名（4 轴模型 link4，6 轴模型 link6）
 *   task_dof = 任务空间维度（4 轴模型 4，6 轴模型 6）
 *   seed_mode: 0 = 零位形起解, 1 = 真实位形附近起解, 2 = 目标加微小扰动
 *
 * @note 该可执行文件只用于离线调参，不参与控制器运行。
 */

#include "default4dof_task.hpp"
#include "default6dof_task.hpp"
#include "kinamic.hpp"
#include "model_from_urdf.hpp"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

namespace {

/// 采样关节角时避开限位端点，避免生成边界上的病态用例。
constexpr double kSampleLowerFraction = 0.15;
constexpr double kSampleUpperFraction = 0.85;

/// 判定「求解成功」的残余误差阈值，比求解器自身的 tolerance 宽松，
/// 用于反映「实际可用」而非「数值上收敛」。
constexpr double kUsableErrorNorm = 1e-4;

struct Case {
    Eigen::VectorXd target;      ///< 任务空间目标
    Eigen::VectorXd true_joints; ///< 生成该目标的真实关节角
};

/// 在限位内均匀采样关节角
Eigen::VectorXd sample_joints(
    const Eigen::VectorXd& lower, const Eigen::VectorXd& upper, std::mt19937* engine) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Eigen::VectorXd q(lower.size());
    for (Eigen::Index i = 0; i < lower.size(); ++i) {
        const double lo = lower(i) + kSampleLowerFraction * (upper(i) - lower(i));
        const double hi = lower(i) + kSampleUpperFraction * (upper(i) - lower(i));
        q(i)            = lo + unit(*engine) * (hi - lo);
    }
    return q;
}

/// 打印一行扫描结果
void report(
    const std::string& tag, int total, int converged, int usable, long long iterations,
    double worst_error, double elapsed_ms) {
    printf(
        "%-46s cases=%4d  converged=%5.1f%%  usable=%5.1f%%  avg_iter=%6.1f  worst_err=%.3e  %6.1f ms\n", tag.c_str(),
        total, 100.0 * converged / total, 100.0 * usable / total, static_cast<double>(iterations) / total, worst_error,
        elapsed_ms);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 4) {
        fprintf(
            stderr,
            "usage: ik_benchmark <urdf> <frame> <task_dof> [cases] [lambda0] [trust0] [trust_grow] [trust_shrink] "
            "[seed_mode]\n"
            "  seed_mode: 0 = 零位形起解, 1 = 真实位形附近起解, 2 = 目标加微小扰动\n");
        return 1;
    }

    const std::string urdf_path = argv[1];
    const std::string frame     = argv[2];
    const int task_dof          = std::atoi(argv[3]);
    const int case_count        = argc > 4 ? std::atoi(argv[4]) : 300;

    IKSolver::Tuning& tuning = IKSolver::tuning();
    if (argc > 5) tuning.lambda_initial = std::atof(argv[5]);
    if (argc > 6) tuning.trust_initial  = std::atof(argv[6]);
    if (argc > 7) tuning.trust_grow     = std::atof(argv[7]);
    if (argc > 8) tuning.trust_shrink   = std::atof(argv[8]);
    const int seed_mode = argc > 9 ? std::atoi(argv[9]) : 1;

    std::shared_ptr<ModelBase> model;
    try {
        model = std::make_shared<ModelFromURDF>(urdf_path, frame);
    } catch (const std::exception& error) {
        fprintf(stderr, "failed to load model: %s\n", error.what());
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

    const Eigen::VectorXd& lower = model->lower_joint_limit();
    const Eigen::VectorXd& upper = model->upper_joint_limit();

    std::mt19937 engine(20261008u);
    std::vector<Case> cases;
    cases.reserve(static_cast<std::size_t>(case_count));
    for (int i = 0; i < case_count; ++i) {
        Case item;
        item.true_joints = sample_joints(lower, upper, &engine);
        Eigen::VectorXd pose(task_dof);
        if (!solver.task_position(item.true_joints, &pose)) {
            continue;
        }
        item.target = pose;
        cases.push_back(std::move(item));
    }

    // 初值：零位形，或真实位形附近加扰动
    std::normal_distribution<double> perturb(0.0, 0.15);
    Eigen::VectorXd seed;
    if (seed_mode == 0) {
        seed = Eigen::VectorXd::Zero(lower.size());
    }

    // seed_mode 2：复现「目标由外部给出、与自身正解略有出入」的情形——例如任务层
    // 用 TF 读末端位姿时，与控制器自身正运动学存在小幅偏差。
    // 做法：把目标在任务空间里做微小扰动，初值取真实关节角（不加噪声）。
    // 位置分量扰 5 mm；姿态分量（6 维下的旋转向量 / 4 维下的 joint4 自转角）扰 0.02。
    if (seed_mode == 2) {
        std::uniform_real_distribution<double> unit(-1.0, 1.0);
        for (Case& item : cases) {
            for (int k = 0; k < 3 && k < task_dof; ++k) {
                item.target(k) += 0.005 * unit(engine);
            }
            for (int k = 3; k < task_dof; ++k) {
                item.target(k) += 0.02 * unit(engine);
            }
        }
    }

    const auto start = std::chrono::steady_clock::now();
    int converged = 0;
    int usable    = 0;
    long long iterations = 0;
    double worst_error = 0.0;

    for (const Case& item : cases) {
        Eigen::VectorXd guess = (seed_mode == 0) ? seed : item.true_joints;
        if (seed_mode == 1) {
            for (Eigen::Index i = 0; i < guess.size(); ++i) {
                guess(i) += perturb(engine);
            }
        }

        Eigen::VectorXd solution = guess;
        const bool ok            = solver.solve(item.target, solution) == IKSolver::SolveResult::Converged;
        const double error_norm  = solver.last_error_norm();
        iterations += solver.last_iterations();
        worst_error = std::max(worst_error, error_norm);
        if (ok) {
            ++converged;
        }
        if (error_norm < kUsableErrorNorm) {
            ++usable;
        }
    }

    const auto finish = std::chrono::steady_clock::now();
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(finish - start).count();

    char tag[256];
    snprintf(
        tag, sizeof(tag), "task_dof=%d lambda0=%.0e trust0=%.2f grow=%.2f shrink=%.2f seed=%s", task_dof,
        tuning.lambda_initial, tuning.trust_initial, tuning.trust_grow, tuning.trust_shrink,
        seed_mode == 0 ? "zero" : "near");
    report(tag, static_cast<int>(cases.size()), converged, usable, iterations, worst_error, elapsed_ms);
    return 0;
}
