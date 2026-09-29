#pragma once

#include "services/parameter_sandbox.h"
#include "services/upgrade_plan.h"
#include "services/workbench_session.h"
#include <QElapsedTimer>
#include <QMainWindow>
#include <optional>

class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QStackedWidget;
class QTableWidget;
class QTextEdit;
class QTimer;
class QCheckBox;
class QDoubleSpinBox;
class QSpinBox;
class QSlider;

namespace yg::host
{
class ScopeWidget;
/** 页面只拥有输入编辑及呈现状态；校验、参数值和保存作业分别由服务持有。 */
class MaintenanceWindow : public QMainWindow
{
  public:
    /**
     * @brief 创建完整调试工作台及维护页面，默认未连接，不创建物理传输。
     * @param parent Qt 父窗口，可为空。
     * @note GUI 线程调用；关闭时停止演示定时器，不能触发设备操作。
     */
    explicit MaintenanceWindow(QWidget *parent = nullptr);
    /**
     * @brief 为本地预览和离线交互检查推进演示会话。
     * @param seconds 模型间隔秒，范围 0..0.2；不操作设备。
     */
    void Window_AdvanceDemo(double seconds);

  private:
    void build_shell();
    QWidget *build_connection_page();
    QWidget *build_home_page();
    QWidget *build_workbench_page();
    QWidget *build_task_page(bool identification);
    QWidget *build_diagnostics_page();
    void show_page(int page);
    void connect_demo();
    void refresh_workbench();
    void refresh_quick_parameter();
    bool parameter_writable() const;
    void export_capture();
    void import_capture();
    void export_task();
    QWidget *build_upgrade_page();
    QWidget *build_parameter_page();
    void select_mode(UpgradeMode mode);
    void invalidate_plan();
    void load_samples();
    void inspect_files();
    void export_job();
    void start_demo();
    void cancel_demo();
    void read_parameters();
    void write_parameter();
    void save_parameters();
    void export_parameters();
    void set_parameter_status(const QString &text);
    UpgradeMode mode_ = UpgradeMode::App;
    bool synthetic_ = false;
    std::optional<UpgradePlan> plan_;
    ParameterSandbox sandbox_;
    QMap<int, ParameterValue> parameter_snapshots_;
    QComboBox *connection_ = nullptr;
    QStackedWidget *pages_ = nullptr;
    QLabel *validation_ = nullptr;
    QLabel *route_ = nullptr;
    QLabel *upgrade_status_ = nullptr;
    QLabel *parameter_status_ = nullptr;
    QLabel *revision_label_ = nullptr;
    QLineEdit *loader_path_ = nullptr;
    QLineEdit *app_path_ = nullptr;
    QVector<QPushButton *> mode_buttons_;
    QPushButton *export_job_button_ = nullptr;
    QPushButton *demo_button_ = nullptr;
    QPushButton *cancel_button_ = nullptr;
    QPushButton *read_button_ = nullptr;
    QPushButton *write_button_ = nullptr;
    QPushButton *save_button_ = nullptr;
    QProgressBar *progress_ = nullptr;
    QTextEdit *upgrade_log_ = nullptr;
    QTableWidget *parameter_table_ = nullptr;
    QTimer *demo_timer_ = nullptr;
    int demo_step_ = 0;
    struct TaskWidgets
    {
        QComboBox *kind = nullptr;
        QPushButton *start = nullptr, *cancel = nullptr, *apply = nullptr, *export_result = nullptr;
        QLabel *phase = nullptr, *result = nullptr;
        QProgressBar *progress = nullptr;
    };
    WorkbenchSession session_;
    QElapsedTimer elapsed_;
    qint64 previous_ms_ = 0;
    ScopeWidget *scope_ = nullptr;
    QComboBox *scenario_ = nullptr, *motion_mode_ = nullptr, *quick_parameter_ = nullptr;
    QSpinBox *node_ = nullptr;
    QDoubleSpinBox *motion_target_ = nullptr;
    QCheckBox *unlock_ = nullptr, *freeze_ = nullptr;
    QLineEdit *quick_edit_ = nullptr;
    QLabel *global_status_ = nullptr, *footer_ = nullptr, *telemetry_ = nullptr;
    QLabel *quick_active_ = nullptr, *quick_status_ = nullptr, *record_status_ = nullptr,
           *task_summary_ = nullptr;
    QPushButton *stop_ = nullptr, *connect_button_ = nullptr, *disconnect_button_ = nullptr;
    QPushButton *enable_ = nullptr, *execute_ = nullptr, *stream_start_ = nullptr,
                *record_start_ = nullptr;
    QPushButton *quick_apply_ = nullptr;
    QSlider *replay_slider_ = nullptr;
    QTextEdit *events_ = nullptr;
    QMap<QString, QLabel *> identity_;
    QVector<QPushButton *> navigation_;
    TaskWidgets task_widgets_[2];
    std::vector<ScopeSample> replay_samples_;
};
} // namespace yg::host
