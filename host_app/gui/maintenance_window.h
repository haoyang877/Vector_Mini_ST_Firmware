#pragma once

#include "services/parameter_sandbox.h"
#include "services/device_readback.h"
#include "services/upgrade_plan.h"
#include "services/workbench_session.h"
#include <QElapsedTimer>
#include <QMainWindow>
#include <optional>

class QComboBox;
class QLabel;
class QLineEdit;
class QProgressBar;
class QProcess;
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
    /** @brief 关闭显式打开的只读辅助进程并释放 CAN 适配器。 */
    ~MaintenanceWindow() override;
    /**
     * @brief 为本地预览和离线交互检查推进演示会话。
     * @param seconds 模型间隔秒，范围 0..0.2；不操作设备。
     */
    void Window_AdvanceDemo(double seconds);
    /**
     * @brief 登记设备端口已发出的只读查询，供真实端口和离线帧回放共用。
     * @param type 1、2、3、108 或 110；GUI 只发送前四种 GET，不发送 110。
     * @param sequence 本连接的非零请求序号。
     * @param page Type 1 的页号，其余为 0。
     * @return 只读 CAN 页面且无演示连接时返回登记结果；不发 CAN 帧。
     */
    bool Window_BeginDeviceRead(uint16_t type, uint16_t sequence, uint16_t page);
    /**
     * @brief 接收完整 CAN FD 数据区并更新已关联的只读视图。
     * @param can_id 经端口检查为扩展 FD/BRS 的 29 位标识符。
     * @param frame 数据区，含 DLC 补零。
     * @return 完整校验并匹配待办请求时返回 true。
     */
    bool Window_ObserveDeviceFrame(uint32_t can_id, const QByteArray &frame);

  private:
    void build_shell();
    QWidget *build_connection_page();
    QWidget *build_home_page();
    QWidget *build_workbench_page();
    QWidget *build_task_page(bool identification);
    QWidget *build_diagnostics_page();
    void show_page(int page);
    void connect_demo();
    void connect_device();
    void disconnect_device();
    void issue_device_query();
    void consume_device_stdout();
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
    DeviceReadback device_readback_;
    QElapsedTimer elapsed_;
    qint64 previous_ms_ = 0;
    ScopeWidget *scope_ = nullptr;
    QComboBox *scenario_ = nullptr, *motion_mode_ = nullptr, *quick_parameter_ = nullptr;
    QSpinBox *node_ = nullptr;
    QLineEdit *python_path_ = nullptr, *sdk_path_ = nullptr;
    QProcess *device_reader_ = nullptr;
    QByteArray device_output_;
    QString device_error_;
    bool device_ready_ = false, device_pending_ = false;
    uint16_t device_sequence_ = 0, device_pending_type_ = 0;
    int device_step_ = 0;
    qint64 device_connect_ms_ = 0, device_request_ms_ = 0, device_discovery_ms_ = 0,
           device_feedback_ms_ = -1000;
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
