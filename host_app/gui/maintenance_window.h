#pragma once

#include "services/device_readback.h"
#include "services/upgrade_plan.h"
#include <QElapsedTimer>
#include <QJsonObject>
#include <QMainWindow>
#include <optional>

class QCheckBox;
class QCloseEvent;
class QLabel;
class QLineEdit;
class QProcess;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTextEdit;

namespace yg::host
{
/** 单页真实设备工具；GUI 只拥有输入与显示，默认不打开端口。 */
class MaintenanceWindow : public QMainWindow
{
  public:
    /** @brief 构建默认断开的 CAN FD 台架工具。 @param parent Qt 父窗口。 */
    explicit MaintenanceWindow(QWidget *parent = nullptr);
    /** @brief 释放显式打开的设备端口。 */
    ~MaintenanceWindow() override;
    /** @brief 登记唯一 GET 请求。 @param type 消息类型。 @param sequence 请求序号。 @param page Type 1 页号。 @return 登记成功返回 true。 */
    bool Window_BeginDeviceRead(uint16_t type, uint16_t sequence, uint16_t page);
    /** @brief 消费经端口校验的 CAN FD 回复。 @param can_id 扩展 CAN ID。 @param frame 完整数据区。 @return 关联并校验成功返回 true。 */
    bool Window_ObserveDeviceFrame(uint32_t can_id, const QByteArray &frame);

  private:
    void closeEvent(QCloseEvent *event) override;
    void build_shell();
    void connect_device();
    void disconnect_device();
    void issue_device_query();
    void consume_device_stdout();
    bool device_identity_matches() const;
    void bind_device_identity();
    bool reserve_control_sequence(bool stopping);
    bool device_safe_idle() const;
    void queue_device_command(const QString &operation, int rpm = 0);
    void append_device_trace(const QString &event, QJsonObject fields = {});
    void inspect_files();
    void start_real_upgrade();
    void consume_upgrade_stdout();
    void refresh_workbench();

    DeviceReadback device_readback_;
    QElapsedTimer elapsed_;
    std::optional<UpgradePlan> plan_;
    QSpinBox *node_ = nullptr, *device_sequence_start_ = nullptr, *device_speed_rpm_ = nullptr;
    QLineEdit *python_path_ = nullptr, *sdk_path_ = nullptr, *app_path_ = nullptr;
    QProcess *device_reader_ = nullptr, *upgrade_process_ = nullptr;
    QByteArray device_output_, upgrade_output_;
    QString upgrade_output_dir_, upgrade_expected_uid_, device_trace_path_, device_error_;
    QString device_command_queued_, device_control_result_, device_expected_uid_;
    bool device_identity_changed_ = false, device_sequence_ready_ = false;
    bool device_sequence_storage_ok_ = true;
    uint16_t device_control_sequence_ = 0;
    bool device_ready_ = false, device_pending_ = false;
    bool device_motion_may_run_ = false, device_stop_ack_ = false, device_start_rejected_ = false;
    int device_command_rpm_ = 0;
    uint16_t device_sequence_ = 0, device_pending_type_ = 0;
    int device_step_ = 0;
    qint64 device_connect_ms_ = 0, device_request_ms_ = 0, device_discovery_ms_ = 0,
           device_feedback_ms_ = -1000, device_stop_deadline_ms_ = 0, device_stop_retry_ms_ = 0;
    QLabel *global_status_ = nullptr, *device_status_ = nullptr, *device_control_status_ = nullptr;
    QLabel *upgrade_status_ = nullptr, *validation_ = nullptr;
    QLabel *uid_value_ = nullptr, *version_value_ = nullptr, *loader_value_ = nullptr;
    QLabel *mode_value_ = nullptr, *fault_value_ = nullptr, *speed_value_ = nullptr;
    QLabel *iq_value_ = nullptr, *bus_value_ = nullptr;
    QCheckBox *device_bench_ack_ = nullptr;
    QPushButton *connect_button_ = nullptr, *disconnect_button_ = nullptr;
    QPushButton *stop_ = nullptr, *device_start_button_ = nullptr, *device_speed_button_ = nullptr;
    QPushButton *upgrade_run_button_ = nullptr;
    QProgressBar *progress_ = nullptr;
    QTextEdit *upgrade_log_ = nullptr;
};
} // namespace yg::host
