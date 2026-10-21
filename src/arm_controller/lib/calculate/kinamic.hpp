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
         * @brief 位置收敛判据（米）
         *
         * 判据不能按纯数值精度来定：
         * - 双精度下迭代会停在 1e-6 量级（舍入 + 雅可比条件数）；
         * - 更关键的是目标位姿往往与控制器自身正解存在微小不一致——任务层经 TF
         *   读取的末端位姿、实机上的标定误差，都会让目标落在可达集之外一点点，
         *   此时误差无论如何都降不到零。实测该不一致约 1.5e-4。
         *
         * 取 1e-3 对应 1 mm，对机械臂控制足够，且给上述不一致留出余量。
         */
        double position_tolerance{1e-3};

        /**
         * @brief 姿态收敛判据（弧度）
         *
         * 与位置分开判，**不再把米和弧度混进同一个范数**：混算时「1e-3」的物理含义
         * 取决于量纲配比，reset_then_cart 这类判据边界用例（位置残余 1.53e-3）就没法
         * 解释。4 维任务空间下这一项只包含 joint4 自转分量。
         */
        double rotation_tolerance{1e-3};

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

    /**
     * @brief 逆解结果
     *
     * 只返回 `bool` 时无法区分「目标不可达」与「数值失败」：两者在日志里都表现为
     * 一句「逆解失败」，操作者看不出是哪一种，也就无从判断该改目标还是该查求解器
     * （docs/bugs.md O2）。
     */
    enum class SolveResult {
        Converged,   ///< 位置与姿态残余同时进入各自判据（@ref Tuning）
        Unreachable, ///< 迭代结束（停滞或次数用尽）时残余仍超判据：目标没有达到，通常意味着落在可达集之外
        Failed,      ///< 数值失败（输入非法、雅可比不可用、步长非有限），可达性未知
    };

    IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping);

    /**
     * @brief 求解逆运动学
     * @param target    任务空间目标（维度由映射决定，见 @ref TaskMapping::task_dim）
     * @param joint_pos 既作输入又作输出：输入为迭代初值（应传当前实测关节角），
     *                  求解失败时内容保持不变
     * @return @ref SolveResult，只有 Converged 表示残余已进入判据
     * @note 判据是任务空间误差范数。目标落在可达集之外时误差降不到零，迭代会在
     *       可达集边界停滞，此时返回 @ref SolveResult::Unreachable 并用
     *       @ref last_error_norm 给出「最近可达位姿离目标还有多远」。
     */
    SolveResult solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos);

    /**
     * @brief 任务误差的 Gauss-Newton 雅可比
     * @param joint_pos     当前关节角
     * @param error         当前任务空间误差（见 @ref TaskMapping::error_map）
     * @param task_jacobian 输出：task_dim × dof
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

    /// @return 任务空间维度（由映射决定：4 轴模型是 4，6 轴模型是 6）
    int task_dim() const { return task_dim_; }

    /// @return 上一次 solve() 结束时的任务空间误差范数（位置与姿态的混合范数），用于诊断
    double last_error_norm() const { return last_error_norm_; }

    /// @return 上一次 solve() 结束时误差的位置分量范数（米）
    double last_position_error_norm() const { return last_position_error_norm_; }

    /// @return 上一次 solve() 结束时误差的姿态分量范数（弧度）
    double last_rotation_error_norm() const { return last_rotation_error_norm_; }

    /// @return 上一次 solve() 实际使用的迭代次数，用于诊断
    int last_iterations() const { return last_iterations_; }

    /// @return 上一次 solve() 的结果，用于区分「不可达」与「数值失败」
    SolveResult last_result() const { return last_result_; }

private:
    /// 计算给定关节角下的任务空间误差
    bool evaluate_error(const Eigen::VectorXd& target, const Eigen::VectorXd& joint_pos, Eigen::VectorXd* error);

    /// 位置与姿态是否**同时**进入各自的判据（见 @ref Tuning）
    bool within_tolerance(const Eigen::VectorXd& error) const;

    /// 记录本次误差的总范数与位置/姿态分量范数，供日志与诊断使用
    void record_error(const Eigen::VectorXd& error);

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
    int task_dim_{6};
    double last_error_norm_{0.0};
    double last_position_error_norm_{0.0};
    double last_rotation_error_norm_{0.0};
    int last_iterations_{0};
    SolveResult last_result_{SolveResult::Failed};
};
