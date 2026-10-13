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
 * 记录 时间 / 位置 / 速度 / **加速度** / 力矩。加速度必须是**真实测量值**
 * （仿真里是 MuJoCo 的 qacc），不能用"对记录速度二次求导"代替——那样相对误差
 * 31.5%，会把辨识结果推出物理可行集。时间列同样记**仿真时间**而不是墙钟，
 * 否则时间戳抖动（实测 dt 标准差 18.7%）会再污染一次求导。
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
