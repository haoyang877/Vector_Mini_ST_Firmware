#pragma once

#include "services/parameter_sandbox.h"
#include "services/upgrade_plan.h"
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

namespace yg::host
{
/** 页面只拥有输入编辑及呈现状态；校验、参数值和保存作业分别由服务持有。 */
class MaintenanceWindow : public QMainWindow
{
  public:
    /**
     * @brief 创建升级与参数页面，默认离线，不创建物理传输。
     * @param parent Qt 父窗口，可为空。
     * @note GUI 线程调用；关闭时停止演示定时器，不能触发设备操作。
     */
    explicit MaintenanceWindow(QWidget *parent = nullptr);

  private:
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
    QLabel *title_ = nullptr;
    QLabel *subtitle_ = nullptr;
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
};
} // namespace yg::host
