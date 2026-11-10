/**
 * @file joint_console_ui.hpp
 * @brief joint_console 的 Qt5 输入窗口
 *
 * 只负责「收集目标角度/执行时间」并把它们写入 @c ConsoleQueue；限位、单步、
 * 起点实测、状态切换与 stop 语义仍由 @c TaskExecutor::run_joint_console 统一处理。
 * 因此终端版与 UI 版的运动行为完全一致。
 *
 * @note 本类不继承 QObject，不使用 moc；窗口与控件都是普通 Qt Widgets。
 */

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

class QApplication;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QWidget;

class JointConsoleUi {
public:
    /// 一行文本的出口（由 TaskExecutor 注入 ConsoleQueue::push_back）
    using LineSink = std::function<void(const std::string&)>;

    /**
     * @brief 构造窗口
     * @param joint_names 关节名（按控制顺序）
     * @param joint_min   每关节下限（弧度）；未配置用 -inf
     * @param joint_max   每关节上限（弧度）；未配置用 +inf
     * @param max_step_rad 单步最大变化量（弧度）；0 = 不限制
     * @param default_duration 执行时间默认值（秒）
     * @param line_sink   输入行出口
     */
    JointConsoleUi(const std::vector<std::string>& joint_names, const std::vector<double>& joint_min,
                   const std::vector<double>& joint_max, double max_step_rad, double default_duration,
                   LineSink line_sink);
    ~JointConsoleUi();

    /// @return Qt 窗口是否可用（无 DISPLAY 时为 false）
    bool available() const { return available_; }

    /// @return 不可用原因
    const std::string& error() const { return error_; }

    void show();
    void hide();

    /// 泵一次 Qt 事件队列（不阻塞）；由 run_joint_console 的等待循环调用
    void process_events();

    /// 更新实时当前角度（弧度）
    void set_current_positions(const std::vector<double>& positions);

    /// 运动中禁用 Execute、启用 Stop
    void set_moving(bool moving);

    /// 状态栏文本；error=true 显示红色
    void set_status(const std::string& text, bool error);

    /// 把目标输入框填成当前实测角度
    void sync_targets_to_current();

    /// 窗口是否已被用户关闭/Exit（步骤应结束）
    bool closed() const { return closed_; }

private:
    void build_ui();
    void on_execute();
    void on_stop();
    void on_exit();
    void on_close();
    void update_current_labels();
    void push_line(const std::string& line);

    std::vector<std::string> joint_names_;
    std::vector<double> joint_min_;
    std::vector<double> joint_max_;
    double max_step_rad_{0.0};
    double default_duration_{3.0};
    LineSink line_sink_;

    bool available_{false};
    bool closed_{false};
    bool moving_{false};
    std::string error_;

    std::unique_ptr<QApplication> app_;
    std::unique_ptr<QWidget> window_;
    std::vector<QLabel*> current_labels_;
    std::vector<QDoubleSpinBox*> target_spins_;
    QDoubleSpinBox* duration_spin_{nullptr};
    QPushButton* execute_button_{nullptr};
    QPushButton* stop_button_{nullptr};
    QLabel* status_label_{nullptr};
    std::vector<double> current_positions_;
};
