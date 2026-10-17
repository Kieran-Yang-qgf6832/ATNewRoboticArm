#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <limits>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
// O12：碰撞检测。注意 PINOCCHIO_WITH_COLLISION 必须由构建系统统一定义，否则
// GeometryData 的 collisionResults / distanceResults 成员根本不存在。
#include <coal/BV/AABB.h>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <vector>

#include "fourier_trajectory.hpp"

class ExcitationTrajectory {
public:
    /// @param urdf_path          模型 URDF 路径
    /// @param end_effector_frame 末端坐标系名（必须存在于 URDF 中；4 自由度模型是 link4）
    explicit ExcitationTrajectory(const std::string& urdf_path, const std::string& end_effector_frame);
    ~ExcitationTrajectory();
    void generate(double period, int repeat_cnt);
    bool generate_is_finished();
    bool generate_failed() const;

    /**
     * @brief 固定 CMA-ES 的随机种子（docs/bugs.md O10）。
     *
     * libcmaes 的语义是「seed 为 0 就用当前时间当种子」，所以默认 0 意味着
     * **每次启动生成的激励轨迹都不同、无法重放**。传入非 0 值即可复现同一条轨迹，
     * 这是做任何"换一版模型再测一次"对比实验的前提。
     *
     * @param seed 0 = 随机（保持旧行为），非 0 = 固定。
     */
    void set_seed(std::uint64_t seed);

    bool get_target_position(const rclcpp::Duration& time, Eigen::VectorXd& pos);

    int get_available_param_num() const;

    /// 最近一次生成实际使用的种子（0 表示由 libcmaes 按时间随机）。
    std::uint64_t get_seed() const;
    /// CMA-ES 原始最优解的软限位违例（缩放前），单位 rad；未生成时为 NaN。
    double get_raw_violation() const;
    /// 为满足约束而施加的整体缩放系数（1.0 = 无需缩放）。
    double get_applied_scale() const;
    /// 最终轨迹的软限位违例，应恒为 0；非 0 表示生成失败。
    double get_final_violation() const;
    /// 最终轨迹的激励得分（越大越好）；失败时为 NaN。
    double get_final_score() const;
    /// 最近一次生成的**碰撞类**违例（自碰撞 + 地面净空，无量纲平方和）。
    ///
    /// 与 get_final_violation() 的分量不同：后者是全部约束的累加，前者只统计碰撞部分，
    /// 用来回答「碰撞约束到底有没有在约束优化」（O12 的观测项）。
    double get_collision_violation() const;

    double end_effector_x_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_x_upper_limit{std::numeric_limits<double>::infinity()};
    double end_effector_y_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_y_upper_limit{std::numeric_limits<double>::infinity()};
    double end_effector_z_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_z_upper_limit{std::numeric_limits<double>::infinity()};

    /// 指令力矩可行性：RNEA 算出的 |关节目力矩| 不得超过 model_.effortLimit 的这个比例
    /// （<= 0 表示关闭）。实机上力矩超限会直接导致跟踪失败；仿真里会把关节甩到高速，
    /// 记录到的 qacc 出现尖峰（实测最大 7375 rad/s²），辨识数据报废。
    double traj_torque_limit_rate{0.0};

    /// 末端**工具最低点**相对末端 frame 原点的偏移（沿 -z，米）。
    ///
    /// 地面净空约束必须作用在工具最低点上，而不是末端原点：实测 link6 的工具网格比它的
    /// frame 原点低 0.0569 m，只卡原点（z ≥ 0.1）时网格仍会钻进地面 4.6 mm，
    /// MuJoCo 的接触冲量会带来 81.9 N·m 的模型残差——而 URDF 里没有地面，无法解释。
    double end_effector_tool_offset{0.0};

    /// 关节位置软限位比例（相对行程中点，0.7 = 只用中间 70%）。日志里会打出实际生效值。
    double traj_pos_soft_limit_rate{0.7};

    /// O12：非相邻连杆之间的碰撞安全间距（米，<= 0 表示关闭）。
    ///
    /// 动机：CMA-ES 优化出来的激励轨迹会把机械臂扭成自碰撞位形——实测 1000 个
    /// 均匀随机位形里有 11 个自碰撞（link1↔link4/5/6、link2↔link4、base_link↔link3/4）。
    /// 仿真里 MuJoCo 的接触冲量会污染辨识数据，实机上更是直接撞坏。
    ///
    /// 实现：coal 的**带安全裕度的碰撞查询**（security_margin = 本值）。碰撞即表示
    /// 距离 < 本值，按「每个违例对 +1」累加进 traj_constraint_violation()。
    /// 之所以不用距离查询（computeDistances）：实测 22 对稠密 STL 网格一次距离查询
    /// 要 79 ms，1000 采样点 × 1500 次评估 = 33 小时，完全不可用；带裕度的碰撞查询
    /// 因为可以提前退出，约 1.2 ms/次。即使这样也仍然偏贵，所以外面还加了一层
    /// 世界 AABB 预筛（见 traj_constraint_violation()）：AABB 距离 ≥ 安全间距时
    /// 精确距离必然也 ≥ 安全间距，直接跳过。
    ///
    /// 相邻连杆对（joint index 相差 1）被显式排除——它们的网格在关节处本来就互相
    /// 嵌入（实测任意位形下距离恒为 0），MuJoCo 的 scene.xml 也用 contact exclude
    /// 把它们排除了；检查它们只会让可行域恒为空。
    double traj_collision_safe_distance{0.0};

    /// O12：连杆网格与地面的净空（米，<= 0 表示关闭）。
    ///
    /// 地面是 z = 0 的水平面（与 MuJoCo scene.xml 的 ground plane 一致），
    /// 用 coal::Plane + security_margin = 本值 实现「每个连杆网格的最低点必须高于 z = 本值」。
    ///
    /// 比现有的 end_effector_z_lower_limit 强的地方在于它卡**每个连杆网格的最低点**，
    /// 而不是只卡末端一个点：实测 link4/link5/link6 在部分位形下都会低于 z=0。
    ///
    /// base_link 被排除在地面对之外：它的网格本来就从 z = -0.0095 开始（安装座埋进
    /// 地面），而仿真里 base_link 是**静态体**，MuJoCo 不计算静态体之间的接触，
    /// 所以它永远"贴着地面"也没有任何接触力。把它纳入约束会让可行域恒为空。
    ///
    /// 与 end_effector_tool_offset + z_lower_limit 是两套机制：那套现在默认关闭
    /// （见 docs/bugs.md O12），保留作 O11 的回归基准与逃生开关。
    double traj_ground_clearance{0.0};

private:
    std::atomic_bool trajectory_generatefinished{false};
    std::atomic_bool trajectory_generatefailed{false};

    // 评估轨迹可行性和质量
    double traj_score(const FourierTrajectory& traj, double dt, int identifiable_param_num);
    double traj_constraint_violation(const FourierTrajectory& traj, double dt);

    /**
     * @brief 求把系数整体缩放后仍满足全部约束的最大系数（二分）。
     *
     * 可行域是以 0 为内点的**凸集**：q(t) 对傅里叶系数线性，约束是逐关节箱式限位，
     * 因此 λ·c 的违例随 λ∈[0,1] 单调不增。λ=0 必然可行，二分即可保证输出可行。
     * @return 可行区间右端（≤1）；若连 λ=0 都不可行（例如末端箱式约束把原点排除）
     *         则返回 0，调用方应据此判定生成失败。
     */
    double find_feasible_scale(FourierTrajectory& traj, const Eigen::VectorXd& coefficients, double dt);

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex end_effector_frame_id_{0};

    // O12：碰撞检测用的几何模型（URDF 的 <collision> 网格，经 pinocchio::urdf::buildGeom
    // 加载）。只在 ExcitationTrajectory 构造时构建一次，之后只读——CMA-ES 的 1500 次
    // 评估全部复用同一份 GeometryModel/GeometryData，没有任何动态分配。
    //
    // 碰撞对集合：去掉相邻连杆对（parentJoint 相差 1）后，再为每个连杆补一个与地面
    // 平面的对。地面平面是 coal::Plane（法向 +z，位于 z = traj_ground_clearance），
    // 挂在 parentJoint = 0（不随任何关节运动）。
    pinocchio::GeometryModel geometry_model_;
    pinocchio::GeometryData geometry_data_;
    /// geometry_model_ 里连杆-连杆对的起始下标；[0, link_pair_num_) 是连杆间碰撞，
    /// [link_pair_num_, collisionPairs.size()) 是连杆-地面对。
    std::size_t link_collision_pair_num_{0};
    bool geometry_valid_{false};
    /// 地面对应的 GeometryObject 下标（geometry_model_ 的最后一个）。
    std::size_t ground_geometry_index_{0};
    /// 每个几何体的**世界** AABB 缓存：预筛用，避免每个采样点重新分配。
    std::vector<coal::AABB> world_aabbs_;
    /// 最近一次 traj_constraint_violation() 累计的碰撞类违例（生成线程私有，非原子）。
    double collision_violation_{0.0};

    /// 把连杆局部坐标下的 AABB 变换到世界系：8 个角点各自变换后取 min/max。
    /// 只平移不旋转（直接用 oMg 的平移）会低估旋转后的包围范围，不能保证保守。
    static coal::AABB worldAabb(const coal::AABB& local, const pinocchio::SE3& placement);


    // 参数
    const double SVD_ZERO = 1e-6;
    double svd_pos_lower{-2.0};
    double svd_pos_upper{2.0};
    double svd_vel_upper{4.0};
    double svd_acc_upper{6.0};
    int svd_Y_stack_num{30}; // 堆叠30个Y矩阵评估矩阵特性

    // O10：随机种子与最近一次生成的诊断量（生成线程写、实时线程读，故用原子量）
    std::atomic<std::uint64_t> seed_{0};
    std::atomic<double> last_raw_violation_{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> last_applied_scale_{1.0};
    std::atomic<double> last_final_violation_{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> last_final_score_{std::numeric_limits<double>::quiet_NaN()};
    // O12：最近一次生成的碰撞类违例（自碰撞 + 地面净空），生成线程写、日志线程读。
    std::atomic<double> last_collision_violation_{std::numeric_limits<double>::quiet_NaN()};

    // 缓存
    Eigen::VectorXd q, dq, ddq;
    Eigen::MatrixXd Y;
    Eigen::VectorXd S;
    Eigen::MatrixXd best_coefficients_;
    Eigen::MatrixXd param_mat;

    int identifiable_parameter_num_{0};

    std::mutex start_calc_mtx_;
    std::condition_variable start_calc_cv_;
    bool start_calc_flag{false};
    std::atomic_bool exit_thread{false};
    void calc_func();

    std::shared_ptr<FourierTrajectory> best_traj;
    int traj_repeat_cnt{1};
    double period_s{10.0};

    std::thread calc_thread_;
};
