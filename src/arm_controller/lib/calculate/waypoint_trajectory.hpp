#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <string>
#include <vector>

/**
 * 多路点七次 Hermite（七次 Bernstein/Bezier）关节空间轨迹，位置/速度/加速度/jerk 连续（C3）。
 *
 * 与 Trajectory 的区别：
 *   - Trajectory 只匹配位置/速度/加速度（五次），且每点的导数由调用方给出；
 *   - 这里只给位置路点，内部用非均匀差商估计中间节点对**归一化弧长**的速度/加速度/jerk，
 *     首尾节点导数固定为 0（静止起停），每段用七次 Bezier 同时匹配两端 p/v/a/j，
 *     因此三阶导数（jerk）在节点处也连续。
 *
 * 时间分配：各段时间正比于该段的弦长，再用一个**全局**时间尺度 T_total 满足
 * 速度/加速度/jerk 限位。这样 Bezier 控制点只由路点几何决定、与 T_total 无关，
 * 避免“逐段独立缩放时长”改变节点差分、把曲线推出关节限位（docs/bugs.md O16）。
 *
 * plan() 只在非实时路径调用；实时路径只调用 sample()，sample() 不做任何内存分配——
 * 所有 buffer 都在 configure() 里按 dof × max_nodes 预分配。
 */
class WaypointTrajectory {
public:
    struct Limits {
        Eigen::VectorXd max_vel;  ///< 各关节速度上限（rad/s），长度 = dof
        Eigen::VectorXd max_acc;  ///< 各关节加速度上限（rad/s²），长度 = dof
        Eigen::VectorXd max_jerk; ///< 各关节 jerk 上限（rad/s³），长度 = dof
    };

    WaypointTrajectory() = default;

    /// 构造期调用：预分配 dof × max_nodes 的全部 buffer，之后 plan/sample 不再分配内存。
    void configure(int dof, int max_nodes);

    /**
     * 规划一条经过全部路点的 C3 轨迹。
     *
     * @param nodes          路点（基座/关节空间位置），数量 ∈ [2, max_nodes]，每个长度 = dof
     * @param limits         速度/加速度/jerk 上限
     * @param total_duration > 0 时把自动最短总时长等比缩放到该值；<= 0 时用自动时长
     * @param lower_limit    关节位置下限（长度 = dof）；与 upper_limit 任一长度不符则不检查
     * @param upper_limit    关节位置上限
     * @param error          失败原因（可为 nullptr）
     * @return 规划成功返回 true
     */
    bool plan(const std::vector<Eigen::VectorXd>& nodes, const Limits& limits, double total_duration,
              const Eigen::VectorXd& lower_limit, const Eigen::VectorXd& upper_limit, std::string* error);

    /**
     * 在时刻 t 采样（t 夹到 [0, duration]）。输出 buffer 必须由调用方按 dof 预分配。
     * 空轨迹返回 false，不做任何内存分配。
     */
    bool sample(double t, Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* acc,
                Eigen::VectorXd* jerk) const;

    [[nodiscard]] double duration() const;
    [[nodiscard]] bool empty() const;
    [[nodiscard]] std::size_t node_count() const;
    /// 第 index 个路点的时刻（index 越界返回 0）。
    [[nodiscard]] double node_time(std::size_t index) const;
    [[nodiscard]] int dof() const;

private:
    // 按当前段时长重算各段起始时刻、节点时刻与总时长。
    void rebuild_times();
    // 用非均匀差商估计中间节点对**归一化弧长**的一/二/三阶导数；首尾节点严格为 0。
    void estimate_derivatives();
    // 用归一化弧长差与节点导数构造每段的 8 个 Bezier 控制点（等价于七次 Hermite）。
    void build_controls();
    // 在段 segment 的归一化参数 u ∈ [0,1] 处解析求值（v/a/j 为对物理时间的导数）。
    void evaluate(std::size_t segment, double u, Eigen::VectorXd* pos, Eigen::VectorXd* vel, Eigen::VectorXd* acc,
                  Eigen::VectorXd* jerk) const;
    // 测量第 segment 段的速度/加速度/jerk 峰值相对限位的比值（> 1 表示超限）。
    double measure_segment_ratio(const Limits& limits, std::size_t segment);

    int dof_ = 0;
    int max_nodes_ = 0;
    std::size_t node_count_ = 0;
    double duration_ = 0.0;

    std::vector<Eigen::VectorXd> node_pos_;  ///< max_nodes × dof
    std::vector<Eigen::VectorXd> node_vel_;  ///< max_nodes × dof，对归一化弧长的导数
    std::vector<Eigen::VectorXd> node_acc_;  ///< max_nodes × dof，对归一化弧长的二阶导
    std::vector<Eigen::VectorXd> node_jerk_; ///< max_nodes × dof，对归一化弧长的三阶导
    std::vector<double> node_u_;             ///< max_nodes，归一化累计弧长
    std::vector<double> node_time_;          ///< max_nodes
    std::vector<Eigen::VectorXd> control_;   ///< (max_nodes-1) × 8 × dof
    std::vector<double> segment_duration_;   ///< max_nodes-1
    std::vector<double> segment_start_;      ///< max_nodes-1

    Eigen::VectorXd work_pos_; ///< 峰值测量的临时输出（预分配，sample 路径不使用）
    Eigen::VectorXd work_vel_;
    Eigen::VectorXd work_acc_;
    Eigen::VectorXd work_jerk_;
    std::vector<double> peak_vel_;
    std::vector<double> peak_acc_;
    std::vector<double> peak_jerk_;
};
