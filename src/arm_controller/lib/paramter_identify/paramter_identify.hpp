#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <Eigen/Dense>

/**
 * @brief 参数辨识测量数据的实时记录器（docs/bugs.md O8）
 *
 * 记录 时间 / 位置 / 速度 / 力矩，**加速度列是可选的**：硬件真的提供 acceleration
 * 状态接口时（目前只有 MuJoCo 仿真）才把 acc_* 写进 CSV，作为诊断量；实机的 MCU
 * 协议只上报位置/速度/力矩，CSV 里就不再有 acc_* 列。
 *
 * 辨识链路已改成不依赖关节加速度（分部积分滤波回归量），所以实机没有加速度也成立。
 * 仿真里依旧记录真实 qacc。时间列记**仿真时间**而不是墙钟（墙钟 dt 抖动实测会让
 * 任何数值微分再污染一次数据）。
 *
 * 记录与落盘分离：record_value() 跑在 500 Hz 实时线程里，只做拷贝；
 * 落盘交给独立线程（save_csv + save_worker_func），避免实时路径做文件 IO。
 */
class ParamterIdentify {
public:
    explicit ParamterIdentify(const std::string& urdf_path);
    ~ParamterIdentify();

    // 参数数量，自由度；预先resize内存，防止运行时分配内存
    void application_memory(int param_num, int dof);

    /**
     * @brief 是否把 acc_* 列写进 CSV（默认写，仿真用）。
     *
     * 在 measure 进入时按硬件是否提供 acceleration 状态接口设置一次；
     * 关闭时 CSV 只有 time, pos_j, vel_j, torque_j 这几组列，辨识链路也不需要加速度。
     * @param enabled true = 写 acc_* 列（仿真）；false = 不写（实机）。
     */
    void set_record_acceleration(bool enabled);

    /**
     * @brief 记录一个采样点（实时线程调用，只做拷贝）。
     * @param pos 关节位置
     * @param vel 关节速度
     * @param acc 关节加速度（真实值，来自 state interface 的 acceleration）
     * @param torque 关节力矩
     * @param time_seconds 当前**仿真时间**（秒）
     * @return 剩余的可记录元素数量；0 表示缓冲区已满或正在落盘（样本被丢弃）
     */
    size_t record_value(const Eigen::VectorXd& pos,
                        const Eigen::VectorXd& vel,
                        const Eigen::VectorXd& acc,
                        const Eigen::VectorXd& torque,
                        double time_seconds);

    bool reset_recording();
    bool save_csv(std::string_view file_name);
    bool save_is_finished() const;
    bool save_succeeded() const;
    std::size_t record_count() const;
    /// 缓冲区容量（= application_memory 传入的 param_num）。
    std::size_t record_capacity() const;

private:
    static constexpr std::size_t kMaxCsvPathLength = 512;

    void save_worker_func();
    bool write_csv_file(std::string_view file_name, std::size_t sample_count) const;

    std::size_t record_index{0};
    Eigen::Index dof_{0};
    /// 是否写 acc_* 列；实机（硬件不报加速度）为 false。
    bool record_acceleration_{true};

    std::atomic_bool save_requested_{false};
    std::atomic_bool save_finished_{true};
    std::atomic_bool save_succeeded_{false};
    std::atomic_bool exit_save_thread_{false};
    std::atomic_size_t save_record_count_{0};
    std::atomic_size_t save_file_name_size_{0};
    std::array<char, kMaxCsvPathLength + 1> save_file_name_{};
    std::thread save_thread_;

    std::vector<double> time_seconds;
    std::vector<Eigen::VectorXd> pos;
    std::vector<Eigen::VectorXd> vel;
    std::vector<Eigen::VectorXd> acc;
    std::vector<Eigen::VectorXd> torque;
};
