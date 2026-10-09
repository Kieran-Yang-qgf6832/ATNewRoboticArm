#pragma once

#include "modelbase.hpp"
#include "task.hpp"
#include <memory>
#include <Eigen/Dense>

/**
 * @brief 阻尼最小二乘逆解器（信赖域 + Levenberg-Marquardt）
 *
 * 迭代策略：
 * -# 用 Levenberg-Marquardt 步 `(JᵀJ + λI)⁻¹Jᵀe` 求步长，λ 随接受/拒绝自适应；
 * -# 用信赖域限制步长范数：步长超过半径就按比例缩回，避免接近奇异时算出巨大步长
 *    再被硬截断导致来回震荡；
 * -# 只有实际误差下降才接受该步，否则缩小信赖域并增大 λ 重试。
 *
 * 参数集中在 @ref Tuning，默认值由离线基准扫描得到（见 tools/ik_benchmark.cpp）。
 */
class IKSolver {
public:
    /**
     * @brief 迭代参数
     *
     * 做成可运行时修改是为了能用离线基准扫描参数；控制器只在启动时使用默认值。
     */
    struct Tuning {
        /**
         * @brief 收敛判据：任务空间误差范数
         *
         * 这是一个「米 + 弧度」混合的 6 维范数，判据不能按纯数值精度来定：
         * - 双精度下迭代会停在 1e-6 量级（舍入 + 雅可比条件数）；
         * - 更关键的是目标位姿往往与控制器自身正解存在微小不一致——任务层经 TF
         *   读取的末端位姿、实机上的标定误差，都会让目标落在可达集之外一点点，
         *   此时误差无论如何都降不到零。实测该不一致约 1.5e-4。
         *
         * 取 1e-3 对应 1 mm / 0.057°，对机械臂控制足够，且给上述不一致留出余量。
         */
        double tolerance{1e-3};
        int max_iterations{200};     ///< 最大迭代次数

        double lambda_initial{1e-2}; ///< LM 阻尼初值（由离线基准扫描确定，见 tools/ik_benchmark.cpp）
        double lambda_up{10.0};      ///< 步被拒绝时的放大倍数
        double lambda_down{0.1};     ///< 步被接受时的缩小倍数
        double lambda_min{1e-9};     ///< LM 阻尼下限
        double lambda_max{1e6};      ///< LM 阻尼上限

        double trust_initial{1.0};   ///< 信赖域半径初值（关节空间范数，弧度；由基准扫描确定）
        double trust_grow{2.0};      ///< 步被接受时的放大倍数
        double trust_shrink{0.25};   ///< 步被拒绝时的缩小倍数
        double trust_min{1e-6};      ///< 信赖域半径下限
        double trust_max{1.0};       ///< 信赖域半径上限

        /**
         * @brief 关节限位容差（弧度）
         *
         * 实测关节角可能略微超出 URDF 限位（柔性、噪声、仿真的软约束）。
         * 限位截断留出该余量，否则「当前位形」本身会变成不可达目标。
         */
        double joint_limit_slack{1e-2};
    };

    /// @return 全局迭代参数（可修改，用于离线调参）
    static Tuning& tuning();

    IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping);

    /**
     * @brief 求解逆运动学
     * @param target    任务空间目标（6 维）
     * @param joint_pos 既作输入又作输出：输入为迭代初值（应传当前实测关节角），
     *                  求解失败时内容保持不变
     * @return true 收敛到 @ref Tuning::tolerance 以内
     */
    bool solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos);

    /**
     * @brief 任务误差的 Gauss-Newton 雅可比
     * @param joint_pos     当前关节角
     * @param error         当前任务空间误差（见 @ref TaskMapping::error_map）
     * @param task_jacobian 输出：6 × dof
     */
    bool jacobian(const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& error, Eigen::MatrixXd* task_jacobian);

    /**
     * @brief 任务坐标雅可比（误差取零）
     *
     * 用于速度映射等不做迭代线性化的场合。误差为零时右雅可比退化为单位阵，
     * 等价于直接把几何雅可比当任务雅可比。
     */
    bool jacobian(const Eigen::VectorXd& joint_pos, Eigen::MatrixXd* task_jacobian);

    bool task_position(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_position);

    /// @return 上一次 solve() 结束时的任务空间误差范数，用于诊断
    double last_error_norm() const { return last_error_norm_; }

    /// @return 上一次 solve() 实际使用的迭代次数，用于诊断
    int last_iterations() const { return last_iterations_; }

private:
    /// 计算给定关节角下的任务空间误差
    bool evaluate_error(const Eigen::VectorXd& target, const Eigen::VectorXd& joint_pos, Eigen::VectorXd* error);

    struct Workspace {
        Eigen::VectorXd solution;
        Eigen::VectorXd current;
        Eigen::VectorXd error;
        Eigen::VectorXd zero_error; ///< 速度映射路径用的零误差（右雅可比退化为单位阵）
        Eigen::VectorXd trial;
        Eigen::VectorXd trial_error;
        Eigen::VectorXd rhs;
        Eigen::VectorXd joint_step;
        Eigen::MatrixXd geometric_jacobian;
        Eigen::MatrixXd task_jacobian;
        Eigen::MatrixXd hessian;
        Eigen::LDLT<Eigen::MatrixXd> ldlt;
    };

    std::shared_ptr<ModelBase> robot_;
    std::shared_ptr<TaskMapping> task_mapping_;
    Workspace workspace_;
    double last_error_norm_{0.0};
    int last_iterations_{0};
};
