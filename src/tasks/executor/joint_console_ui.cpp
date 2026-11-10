/**
 * @file joint_console_ui.cpp
 * @brief joint_console 的 Qt5 输入窗口实现
 */

#include "joint_console_ui.hpp"

#include <QApplication>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QEvent>
#include <QEventLoop>
#include <QFontDatabase>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QShortcut>
#include <QVBoxLayout>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace {

constexpr double kPi      = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

/// 拦截窗口关闭：运动中先 stop，再结束步骤
class CloseFilter : public QObject {
public:
    explicit CloseFilter(std::function<void()> callback) : callback_(std::move(callback)) {}

    bool eventFilter(QObject* watched, QEvent* event) override {
        if (event->type() == QEvent::Close) {
            callback_();
            return true;
        }
        return QObject::eventFilter(watched, event);
    }

private:
    std::function<void()> callback_;
};

QString limits_text(double min_deg, double max_deg) {
    if (!std::isfinite(min_deg) || !std::isfinite(max_deg)) {
        return QStringLiteral("unlimited");
    }
    return QStringLiteral("[%1, %2]").arg(min_deg, 0, 'f', 1).arg(max_deg, 0, 'f', 1);
}

} // namespace

JointConsoleUi::JointConsoleUi(const std::vector<std::string>& joint_names, const std::vector<double>& joint_min,
                               const std::vector<double>& joint_max, double max_step_rad, double default_duration,
                               LineSink line_sink)
    : joint_names_(joint_names),
      joint_min_(joint_min),
      joint_max_(joint_max),
      max_step_rad_(max_step_rad),
      default_duration_(default_duration),
      line_sink_(std::move(line_sink)) {
    if (qEnvironmentVariableIsEmpty("DISPLAY") && qEnvironmentVariableIsEmpty("WAYLAND_DISPLAY")) {
        error_ = "DISPLAY/WAYLAND_DISPLAY is empty; the Qt UI needs a graphical session";
        return;
    }

    if (QApplication::instance() == nullptr) {
        // QApplication 需要 argc/argv 一直有效；用静态存储，生命周期覆盖整个进程。
        static int argc                  = 1;
        static char application_name[]    = "task_runner";
        static char* argv[]              = {application_name, nullptr};
        app_                             = std::make_unique<QApplication>(argc, argv);
        QApplication::setQuitOnLastWindowClosed(false);
    }

    available_ = true;
    build_ui();
}

JointConsoleUi::~JointConsoleUi() {
    // 进程即将退出：不要在这里销毁 QApplication/窗口。
    // 在 ros2 launch 的管道 stdin 环境下，Qt5 的 xcb 清理线程可能在析构时卡住，
    // 导致序列已经结束后 task_runner 不退出、launch 一直挂着。窗口此时已经 hide，
    // 进程退出后由操作系统回收即可。
    std::fprintf(stderr, "[JointConsoleUi] destructor: leaving Qt objects to process exit\n");
    window_.release();
    app_.release();
}

void JointConsoleUi::build_ui() {
    window_ = std::make_unique<QWidget>();
    window_->setWindowTitle(QStringLiteral("Joint Console"));

    auto* main_layout = new QVBoxLayout(window_.get());

    auto* title = new QLabel(QStringLiteral("<b>Joint Console</b>"));
    main_layout->addWidget(title);

    auto* hint = new QLabel(QStringLiteral(
        "输入目标角度（度）与执行时间，点 Execute（或在输入框里按 Enter）下发；"
        "运动中 Stop 会切回 idel 并就地保持。"));
    hint->setWordWrap(true);
    main_layout->addWidget(hint);

    auto* grid = new QGridLayout();
    grid->addWidget(new QLabel(QStringLiteral("Joint")), 0, 0);
    grid->addWidget(new QLabel(QStringLiteral("Current (deg)")), 0, 1);
    grid->addWidget(new QLabel(QStringLiteral("Target (deg)")), 0, 2);
    grid->addWidget(new QLabel(QStringLiteral("Limits (deg)")), 0, 3);

    const QFont fixed_font = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        const auto row = static_cast<int>(i) + 1;

        auto* name_label = new QLabel(QString::fromStdString(joint_names_[i]));
        grid->addWidget(name_label, row, 0);

        auto* current_label = new QLabel(QStringLiteral("--"));
        current_label->setFont(fixed_font);
        current_label->setMinimumWidth(96);
        current_label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        current_labels_.push_back(current_label);
        grid->addWidget(current_label, row, 1);

        const double min_deg = std::isfinite(joint_min_[i]) ? joint_min_[i] / kDegToRad : -360.0;
        const double max_deg = std::isfinite(joint_max_[i]) ? joint_max_[i] / kDegToRad : 360.0;

        auto* target_spin = new QDoubleSpinBox();
        target_spin->setRange(min_deg, max_deg);
        target_spin->setDecimals(2);
        target_spin->setSingleStep(0.1);
        target_spin->setSuffix(QStringLiteral(" °"));
        target_spin->setAlignment(Qt::AlignRight);
        target_spins_.push_back(target_spin);
        grid->addWidget(target_spin, row, 2);

        auto* limit_label = new QLabel(limits_text(min_deg, max_deg));
        grid->addWidget(limit_label, row, 3);
    }

    main_layout->addLayout(grid);

    auto* duration_row = new QHBoxLayout();
    duration_row->addWidget(new QLabel(QStringLiteral("Execution time:")));
    duration_spin_ = new QDoubleSpinBox();
    duration_spin_->setRange(0.1, 60.0);
    duration_spin_->setDecimals(2);
    duration_spin_->setSingleStep(0.5);
    duration_spin_->setValue(std::clamp(default_duration_, 0.1, 60.0));
    duration_spin_->setSuffix(QStringLiteral(" s"));
    duration_row->addWidget(duration_spin_);
    duration_row->addStretch();
    main_layout->addLayout(duration_row);

    auto* button_row = new QHBoxLayout();
    auto* use_current_button = new QPushButton(QStringLiteral("Use current"));
    execute_button_          = new QPushButton(QStringLiteral("Execute"));
    execute_button_->setDefault(true);
    stop_button_ = new QPushButton(QStringLiteral("Stop"));
    stop_button_->setEnabled(false);
    auto* exit_button = new QPushButton(QStringLiteral("Exit"));

    button_row->addWidget(use_current_button);
    button_row->addStretch();
    button_row->addWidget(execute_button_);
    button_row->addWidget(stop_button_);
    button_row->addWidget(exit_button);
    main_layout->addLayout(button_row);

    status_label_ = new QLabel(QStringLiteral("就绪"));
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet(QStringLiteral("color: #1b5e20;"));
    main_layout->addWidget(status_label_);

    QObject::connect(use_current_button, &QPushButton::clicked, window_.get(), [this] { sync_targets_to_current(); });
    QObject::connect(execute_button_, &QPushButton::clicked, window_.get(), [this] { on_execute(); });
    QObject::connect(stop_button_, &QPushButton::clicked, window_.get(), [this] { on_stop(); });
    QObject::connect(exit_button, &QPushButton::clicked, window_.get(), [this] { on_exit(); });

    // 键盘快捷键：Alt+E 执行、Alt+S 停止、Alt+X 退出、Alt+C 取当前角
    auto* execute_shortcut = new QShortcut(QKeySequence(QStringLiteral("Alt+E")), window_.get());
    QObject::connect(execute_shortcut, &QShortcut::activated, window_.get(), [this] { on_execute(); });
    auto* stop_shortcut = new QShortcut(QKeySequence(QStringLiteral("Alt+S")), window_.get());
    QObject::connect(stop_shortcut, &QShortcut::activated, window_.get(), [this] { on_stop(); });
    auto* exit_shortcut = new QShortcut(QKeySequence(QStringLiteral("Alt+X")), window_.get());
    QObject::connect(exit_shortcut, &QShortcut::activated, window_.get(), [this] { on_exit(); });
    auto* current_shortcut = new QShortcut(QKeySequence(QStringLiteral("Alt+C")), window_.get());
    QObject::connect(current_shortcut, &QShortcut::activated, window_.get(), [this] { sync_targets_to_current(); });

    // 普通 QWidget 没有 QDialog 的 default-button 机制，Enter/Return 需要显式接上，
    // 否则提示里的“输入框按 Enter 下发”不会生效。
    auto* return_shortcut = new QShortcut(QKeySequence(Qt::Key_Return), window_.get());
    QObject::connect(return_shortcut, &QShortcut::activated, window_.get(), [this] { on_execute(); });
    auto* enter_shortcut = new QShortcut(QKeySequence(Qt::Key_Enter), window_.get());
    QObject::connect(enter_shortcut, &QShortcut::activated, window_.get(), [this] { on_execute(); });

    auto* close_filter = new CloseFilter([this] { on_close(); });
    close_filter->setParent(window_.get());
    window_->installEventFilter(close_filter);

    window_->resize(640, 430);
}

void JointConsoleUi::show() {
    if (!available_ || window_ == nullptr) {
        return;
    }
    closed_ = false;
    window_->show();
    window_->raise();
    window_->activateWindow();
    process_events();
    sync_targets_to_current();
    set_moving(false);
    set_status("就绪", false);
    if (!target_spins_.empty()) {
        target_spins_.front()->setFocus();
    }
}

void JointConsoleUi::hide() {
    if (window_ != nullptr) {
        window_->hide();
    }
}

void JointConsoleUi::process_events() {
    if (QApplication::instance() != nullptr) {
        QApplication::processEvents(QEventLoop::AllEvents, 10);
    }
}

void JointConsoleUi::set_current_positions(const std::vector<double>& positions) {
    if (positions.size() != joint_names_.size()) {
        return;
    }
    current_positions_ = positions;
    update_current_labels();
}

void JointConsoleUi::update_current_labels() {
    if (current_positions_.size() != current_labels_.size()) {
        return;
    }
    for (std::size_t i = 0; i < current_labels_.size(); ++i) {
        current_labels_[i]->setText(QString::number(current_positions_[i] / kDegToRad, 'f', 2));
    }
}

void JointConsoleUi::set_moving(bool moving) {
    moving_ = moving;
    if (execute_button_ != nullptr) {
        execute_button_->setEnabled(!moving);
    }
    if (stop_button_ != nullptr) {
        stop_button_->setEnabled(moving);
    }
    if (!moving && available_) {
        set_status("就绪", false);
    }
}

void JointConsoleUi::set_status(const std::string& text, bool error) {
    if (status_label_ == nullptr) {
        return;
    }
    status_label_->setText(QString::fromStdString(text));
    status_label_->setStyleSheet(error ? QStringLiteral("color: #b00020;") : QStringLiteral("color: #1b5e20;"));
}

void JointConsoleUi::sync_targets_to_current() {
    if (current_positions_.size() != target_spins_.size()) {
        return;
    }
    for (std::size_t i = 0; i < target_spins_.size(); ++i) {
        target_spins_[i]->setValue(current_positions_[i] / kDegToRad);
    }
}

void JointConsoleUi::push_line(const std::string& line) {
    if (line_sink_) {
        line_sink_(line);
    }
}

void JointConsoleUi::on_execute() {
    if (!available_ || moving_) {
        return;
    }
    if (current_positions_.size() != joint_names_.size()) {
        set_status("还没有收到 /joint_states，无法确定起点", true);
        return;
    }

    std::vector<double> targets_deg;
    targets_deg.reserve(joint_names_.size());
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        const double target_deg = target_spins_[i]->value();
        const double target_rad = target_deg * kDegToRad;

        if (std::isfinite(joint_min_[i]) && target_rad < joint_min_[i] - 1e-9) {
            set_status(joint_names_[i] + " 低于下限，请重输", true);
            return;
        }
        if (std::isfinite(joint_max_[i]) && target_rad > joint_max_[i] + 1e-9) {
            set_status(joint_names_[i] + " 高于上限，请重输", true);
            return;
        }
        if (max_step_rad_ > 0.0 && std::abs(target_rad - current_positions_[i]) > max_step_rad_ + 1e-9) {
            set_status(joint_names_[i] + " 单步变化 "
                           + std::to_string(std::abs(target_rad - current_positions_[i]) / kDegToRad)
                           + " deg 超过上限 " + std::to_string(max_step_rad_ / kDegToRad) + " deg",
                       true);
            return;
        }
        targets_deg.push_back(target_deg);
    }

    const double duration = duration_spin_ != nullptr ? duration_spin_->value() : default_duration_;
    if (!(duration > 0.0)) {
        set_status("执行时间必须为正数", true);
        return;
    }

    for (const double deg : targets_deg) {
        push_line(QString::number(deg, 'f', 6).toStdString());
    }
    push_line(QString::number(duration, 'f', 6).toStdString());

    moving_ = true;
    execute_button_->setEnabled(false);
    stop_button_->setEnabled(true);
    set_status("已下发；运动中可点 Stop", false);
}

void JointConsoleUi::on_stop() {
    if (!moving_) {
        set_status("当前没有运动，无需停止", false);
        return;
    }
    push_line(QStringLiteral("stop").toStdString());
    set_status("已请求停止：控制器将切回 idel 并就地保持", false);
}

void JointConsoleUi::on_exit() {
    if (closed_) {
        return;
    }
    if (moving_) {
        // 运动中退出先 stop，避免轨迹继续跑
        push_line(QStringLiteral("stop").toStdString());
    }
    closed_ = true;
    hide();
}

void JointConsoleUi::on_close() {
    on_exit();
}
