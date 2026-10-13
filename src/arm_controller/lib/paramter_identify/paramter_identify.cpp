#include "paramter_identify.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

ParamterIdentify::ParamterIdentify(const std::string& urdf_path)
    : save_thread_([this]() { save_worker_func(); }) {
    (void)urdf_path;
}

ParamterIdentify::~ParamterIdentify() {
    exit_save_thread_.store(true, std::memory_order_release);
    if (save_thread_.joinable()) {
        save_thread_.join();
    }
}

void ParamterIdentify::application_memory(int param_num, int dof) {
    record_index = 0;
    dof_         = dof > 0 ? dof : 0;

    if (param_num <= 0 || dof_ == 0) {
        time_seconds.clear();
        pos.clear();
        vel.clear();
        acc.clear();
        torque.clear();
        return;
    }

    const auto buffer_size = static_cast<std::size_t>(param_num);
    time_seconds.assign(buffer_size, 0.0);
    pos.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
    vel.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
    acc.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
    torque.assign(buffer_size, Eigen::VectorXd::Zero(dof_));
}

bool ParamterIdentify::reset_recording() {
    if (save_requested_.load(std::memory_order_acquire)) {
        return false;
    }

    record_index = 0;
    return true;
}

size_t ParamterIdentify::record_value(const Eigen::VectorXd& pos,
                                      const Eigen::VectorXd& vel,
                                      const Eigen::VectorXd& acc,
                                      const Eigen::VectorXd& torque,
                                      double time_seconds) {
    if (save_requested_.load(std::memory_order_acquire) || record_index >= this->pos.size() || dof_ == 0) {
        return 0;
    }

    if (pos.size() != dof_ || vel.size() != dof_ || acc.size() != dof_ || torque.size() != dof_) {
        return this->pos.size() - record_index;
    }

    // 时间列记仿真时间而不是墙钟（O8）：墙钟 dt 标准差 18.7%，按它求导会额外放大误差。
    this->time_seconds[record_index] = time_seconds;
    this->pos[record_index]          = pos;
    this->vel[record_index]          = vel;
    this->acc[record_index]          = acc;
    this->torque[record_index]       = torque;
    ++record_index;

    return this->pos.size() - record_index;
}

bool ParamterIdentify::save_csv(std::string_view file_name) {
    if (file_name.empty() || file_name.size() > kMaxCsvPathLength
        || save_requested_.load(std::memory_order_acquire)) {
        return false;
    }

    for (std::size_t i = 0; i < file_name.size(); ++i) {
        save_file_name_[i] = file_name[i];
    }
    save_file_name_[file_name.size()] = '\0';

    save_file_name_size_.store(file_name.size(), std::memory_order_relaxed);
    save_record_count_.store(std::min(record_index, pos.size()), std::memory_order_relaxed);
    save_succeeded_.store(false, std::memory_order_relaxed);
    save_finished_.store(false, std::memory_order_release);
    save_requested_.store(true, std::memory_order_release);
    return true;
}

bool ParamterIdentify::save_is_finished() const {
    return save_finished_.load(std::memory_order_acquire);
}

bool ParamterIdentify::save_succeeded() const {
    return save_succeeded_.load(std::memory_order_acquire);
}

std::size_t ParamterIdentify::record_count() const {
    return record_index;
}

std::size_t ParamterIdentify::record_capacity() const {
    return pos.size();
}

void ParamterIdentify::save_worker_func() {
    while (true) {
        if (save_requested_.load(std::memory_order_acquire)) {
            const std::size_t file_name_size = save_file_name_size_.load(std::memory_order_relaxed);
            const std::size_t sample_count   = save_record_count_.load(std::memory_order_relaxed);
            const std::string file_name(save_file_name_.data(), file_name_size);

            bool save_ok = false;
            try {
                save_ok = write_csv_file(file_name, sample_count);
            } catch (const std::exception& error) {
                std::cerr << "Failed to save parameter identification CSV: " << error.what() << std::endl;
            }

            save_succeeded_.store(save_ok, std::memory_order_release);
            save_requested_.store(false, std::memory_order_release);
            save_finished_.store(true, std::memory_order_release);
            continue;
        }

        if (exit_save_thread_.load(std::memory_order_acquire)) {
            return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool ParamterIdentify::write_csv_file(std::string_view file_name, std::size_t sample_count) const {
    namespace fs = std::filesystem;

    const fs::path csv_path{std::string(file_name)};
    if (csv_path.has_parent_path()) {
        fs::create_directories(csv_path.parent_path());
    }

    std::ofstream stream(csv_path, std::ios::out | std::ios::trunc);
    if (!stream.is_open()) {
        return false;
    }

    stream << std::setprecision(17);
    stream << "time";
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",pos_" << joint;
    }
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",vel_" << joint;
    }
    // 真实加速度列（O8）：csv_loader 用 acceleration_source: recorded 直接读它们，
    // 不再对速度做二次求导。
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",acc_" << joint;
    }
    for (Eigen::Index joint = 0; joint < dof_; ++joint) {
        stream << ",torque_" << joint;
    }
    stream << '\n';

    const std::size_t count =
        std::min({sample_count, time_seconds.size(), pos.size(), vel.size(), acc.size(), torque.size()});
    if (count == 0) {
        return static_cast<bool>(stream);
    }

    // 时间列以第一个样本为 0，单位秒，来自**仿真时间**（O8）。
    const double start_time = time_seconds[0];
    for (std::size_t sample = 0; sample < count; ++sample) {
        stream << (time_seconds[sample] - start_time);
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << pos[sample](joint);
        }
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << vel[sample](joint);
        }
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << acc[sample](joint);
        }
        for (Eigen::Index joint = 0; joint < dof_; ++joint) {
            stream << ',' << torque[sample](joint);
        }
        stream << '\n';
    }

    return static_cast<bool>(stream);
}
