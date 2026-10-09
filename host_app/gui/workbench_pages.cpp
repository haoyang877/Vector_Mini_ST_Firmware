/** 工作区只组合会话与参数服务；维护功能是其中一页，不替换完整工作台。 */
#include "maintenance_window.h"
#include "scope_widget.h"
#include "services/capture_csv.h"
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFile>
#include <QDir>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QProcess>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QStandardItemModel>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>

namespace yg::host
{
namespace
{
QLabel *text_label(const QString &text, const char *name = "subtitle")
{
    auto *item = new QLabel(text);
    item->setObjectName(name);
    return item;
}
QPushButton *action(const QString &text, const char *name, const char *role = "secondary")
{
    auto *item = new QPushButton(text);
    item->setObjectName(name);
    item->setProperty("role", role);
    item->setMinimumHeight(32);
    item->setCursor(Qt::PointingHandCursor);
    return item;
}
QGroupBox *group(const QString &title)
{
    return new QGroupBox(title);
}
} // namespace

void MaintenanceWindow::build_shell()
{
    setWindowTitle(QStringLiteral("YG_MC_HOST v" YG_MC_HOST_VERSION " · 电机调试工作台"));
    resize(1366, 768);
    setMinimumSize(1080, 680);
    auto *root = new QWidget;
    root->setObjectName("workspace");
    auto *layout = new QVBoxLayout(root);
    layout->setContentsMargins(16, 12, 16, 10);
    layout->setSpacing(9);
    auto *top = new QHBoxLayout;
    top->addWidget(text_label(QStringLiteral("YG_MC_HOST v" YG_MC_HOST_VERSION), "brandCompact"));
    top->addSpacing(12);
    global_status_ = text_label("未连接", "globalStatus");
    top->addWidget(global_status_, 1);
    top->addWidget(text_label("离线演示", "badge"));
    auto *connection_settings = action("连接设置", "connectionSettings");
    disconnect_button_ = action("断开", "disconnectDevice");
    stop_ = action("停止并失能", "stopMotor", "danger");
    top->addWidget(connection_settings);
    top->addWidget(disconnect_button_);
    top->addWidget(stop_);
    layout->addLayout(top);
    auto *notice = text_label(
        "演示数据与真实设备隔离 · CAN FD 速度/APP升级仅本轮架空台架候选，尚非产品开放能力",
        "notice");
    layout->addWidget(notice);
    auto *nav = new QHBoxLayout;
    auto *nav_group = new QButtonGroup(this);
    const QStringList names{
        "首页", "调试工作台", "参数配置", "标定", "辨识", "记录与诊断", "设备维护"};
    const QStringList ids{"homeNav",
                          "workbenchNav",
                          "parameterNav",
                          "calibrationNav",
                          "identificationNav",
                          "diagnosticsNav",
                          "upgradeNav"};
    for (int i = 0; i < names.size(); ++i)
    {
        auto *item = action(names[i], qPrintable(ids[i]), "tab");
        item->setCheckable(true);
        nav_group->addButton(item);
        navigation_.append(item);
        nav->addWidget(item);
        connect(item,
                &QPushButton::clicked,
                this,
                [this, i]
                {
                    show_page(i + 1);
                });
    }
    nav->addStretch();
    layout->addLayout(nav);
    pages_ = new QStackedWidget;
    pages_->setObjectName("workspacePages");
    pages_->addWidget(build_connection_page());
    pages_->addWidget(build_home_page());
    pages_->addWidget(build_workbench_page());
    pages_->addWidget(build_parameter_page());
    pages_->addWidget(build_task_page(false));
    pages_->addWidget(build_task_page(true));
    pages_->addWidget(build_diagnostics_page());
    pages_->addWidget(build_upgrade_page());
    layout->addWidget(pages_, 1);
    footer_ = text_label("未连接；连接不会自动使能或启动采集。", "footer");
    layout->addWidget(footer_);
    setCentralWidget(root);
    connect(connection_settings,
            &QPushButton::clicked,
            this,
            [this]
            {
                show_page(0);
            });
    connect(disconnect_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                if (upgrade_process_)
                {
                    return;
                }
                if (device_motion_may_run_ && device_reader_)
                {
                    queue_device_command("stop");
                    return;
                }
                session_.Session_Disconnect();
                disconnect_device();
                cancel_demo();
                unlock_->setChecked(false);
                refresh_workbench();
            });
    connect(stop_,
            &QPushButton::clicked,
            this,
            [this]
            {
                if (device_reader_)
                {
                    queue_device_command("stop");
                }
                else
                {
                    session_.Session_Stop();
                }
                refresh_workbench();
            });
    connect(connection_,
            &QComboBox::currentIndexChanged,
            this,
            [this]
            {
                session_.Session_Disconnect();
                disconnect_device();
                cancel_demo();
                unlock_->setChecked(false);
                freeze_->setChecked(false);
                replay_samples_.clear();
                scope_->Scope_SetSamples({});
                scope_->Scope_SetReplayFraction(std::nullopt);
                device_readback_.Device_Reset(quint8(node_->value()));
                refresh_workbench();
            });
    connect(node_,
            &QSpinBox::valueChanged,
            this,
            [this]
            {
                disconnect_device();
                device_readback_.Device_Reset(quint8(node_->value()));
                refresh_workbench();
            });
    device_readback_.Device_Reset(quint8(node_->value()));
    connect(parameter_table_,
            &QTableWidget::itemChanged,
            this,
            [this](QTableWidgetItem *item)
            {
                if (item->column() == 4)
                {
                    refresh_quick_parameter();
                }
            });
    elapsed_.start();
    auto *timer = new QTimer(this);
    timer->setInterval(20);
    connect(timer,
            &QTimer::timeout,
            this,
            [this]
            {
                const auto now_ms = elapsed_.elapsed();
                session_.Session_Tick(std::min(0.2, double(now_ms - previous_ms_) / 1000.0));
                previous_ms_ = now_ms;
                if (device_reader_ && !device_ready_ && now_ms - device_connect_ms_ > 2000)
                {
                    device_error_ = "CAN 辅助进程未确认端口打开；连接超时。";
                    disconnect_device();
                }
                if (device_pending_ && now_ms - device_request_ms_ > 250)
                {
                    device_error_ = "CAN 请求超时；若已请求运动或 STOP，须人工核对关相。";
                    disconnect_device();
                }
                if (device_reader_ && device_stop_deadline_ms_ > 0 &&
                    now_ms > device_stop_deadline_ms_)
                {
                    device_error_ = "STOP 最终确认超时；保持停机并人工核对关相。";
                    disconnect_device();
                }
                issue_device_query();
                refresh_workbench();
            });
    timer->start();
    show_page(0);
    refresh_workbench();
}

void MaintenanceWindow::show_page(int page)
{
    pages_->setCurrentIndex(page);
    for (int i = 0; i < navigation_.size(); ++i)
    {
        navigation_[i]->setChecked(i + 1 == page);
    }
}

QWidget *MaintenanceWindow::build_connection_page()
{
    auto *page = new QWidget;
    auto *layout = new QHBoxLayout(page);
    layout->setContentsMargins(40, 25, 40, 25);
    layout->setSpacing(35);
    auto *intro = group("开始调试");
    auto *info = new QVBoxLayout(intro);
    info->setSpacing(22);
    info->addWidget(text_label("先连接，再确认设备与状态", "pageTitle"));
    info->addWidget(text_label(
        "01  选择接口与目标节点\n\n02  核对设备与版本信息\n\n03  进入工作台，显式开始采集"));
    info->addStretch();
    auto *explanation =
        text_label("演示连接不会启动电机。实机后端接入后，同样需要按设备能力确认操作权限。");
    explanation->setWordWrap(true);
    info->addWidget(explanation);
    layout->addWidget(intro, 1);
    auto *config = group("连接配置");
    auto *form = new QVBoxLayout(config);
    form->setSpacing(15);
    connection_ = new QComboBox;
    connection_->setObjectName("connectionMode");
    connection_->addItems({"离线演示 · 不连接设备", "CAN FD · 台架候选", "SWD · 待接入"});
    scenario_ = new QComboBox;
    scenario_->setObjectName("demoScenario");
    scenario_->addItems({"正常 APP", "只读 APP", "活动故障 APP", "Loader"});
    node_ = new QSpinBox;
    node_->setObjectName("targetNode");
    node_->setRange(1, 254);
    node_->setValue(3);
    form->addWidget(text_label("通信接口"));
    form->addWidget(connection_);
    form->addWidget(text_label("目标节点"));
    form->addWidget(node_);
    device_sequence_start_ = new QSpinBox;
    device_sequence_start_->setObjectName("benchSequenceStart");
    device_sequence_start_->setRange(0, 65534);
    device_sequence_start_->setValue(0);
    device_sequence_start_->setToolTip(
        "仅依据同一未重启设备的源节点2日志显式设置；下一请求使用起点+1。");
    form->addWidget(text_label("台架序号起点（高级；下一请求为起点 + 1）"));
    form->addWidget(device_sequence_start_);
    form->addWidget(text_label("演示场景"));
    form->addWidget(scenario_);
    python_path_ = new QLineEdit;
    python_path_->setObjectName("pythonRuntimePath");
    python_path_->setPlaceholderText("选择本地 Python 3 运行时");
    sdk_path_ = new QLineEdit;
    sdk_path_->setObjectName("canSdkPath");
    sdk_path_->setPlaceholderText("选择 ControlCANFD SDK DLL");
    form->addWidget(text_label("CAN FD 台架 · Python 运行时"));
    form->addWidget(python_path_);
    form->addWidget(text_label("CAN FD 台架 · SDK DLL"));
    form->addWidget(sdk_path_);
    auto *browse_python = action("选择 Python", "browsePython");
    auto *browse_sdk = action("选择 SDK DLL", "browseSdk");
    auto *paths = new QHBoxLayout;
    paths->addWidget(browse_python);
    paths->addWidget(browse_sdk);
    form->addLayout(paths);
    connect(browse_python,
            &QPushButton::clicked,
            this,
            [this]
            {
                const auto path = QFileDialog::getOpenFileName(this, "选择 Python 3 可执行文件");
                if (!path.isEmpty())
                {
                    python_path_->setText(path);
                }
            });
    connect(browse_sdk,
            &QPushButton::clicked,
            this,
            [this]
            {
                const auto path = QFileDialog::getOpenFileName(
                    this, "选择 ControlCANFD SDK DLL", {}, "DLL (*.dll)");
                if (!path.isEmpty())
                {
                    sdk_path_->setText(path);
                }
            });
    const auto target = Upgrade_Target();
    const auto constants = target["constants"].toObject();
    auto *target_info =
        text_label(QString("本地目标配置：%1\nlayout %2 / schema %3\n实机身份：未识别")
                       .arg(target["device"].toString(),
                            constants["TARGET_LAYOUT"].toString().remove('U'),
                            constants["TARGET_SCHEMA"].toString().remove('U')));
    target_info->setWordWrap(true);
    form->addWidget(target_info);
    form->addStretch();
    connect_button_ = action("连接演示设备", "connectDevice", "primary");
    form->addWidget(connect_button_);
    connect(connect_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                if (connection_->currentIndex() == 0)
                {
                    connect_demo();
                }
                else if (connection_->currentIndex() == 1)
                {
                    connect_device();
                }
            });
    layout->addWidget(config, 1);
    return page;
}

void MaintenanceWindow::connect_demo()
{
    if (connection_->currentIndex() != 0 || session_.Session_View().connected)
    {
        return;
    }
    if (sandbox_.Sandbox_SavePending())
    {
        set_parameter_status("请先等待演示保存作业完成。");
        return;
    }
    const auto scenario = DemoScenario(scenario_->currentIndex());
    if (!session_.Session_Connect(quint8(node_->value()), scenario))
    {
        return;
    }
    sandbox_ = ParameterSandbox(quint8(node_->value()), scenario != DemoScenario::ReadOnly);
    parameter_snapshots_.clear();
    for (int row = 0; row < parameter_table_->rowCount(); ++row)
    {
        parameter_table_->item(row, 2)->setText("未读取");
        parameter_table_->item(row, 4)->setText("");
        parameter_table_->item(row, 5)->setText("未知");
        parameter_table_->item(row, 4)->setFlags(parameter_table_->item(row, 4)->flags() &
                                                 ~Qt::ItemIsEditable);
    }
    revision_label_->setText("活动 revision：未知    |    已保存 revision：未知");
    set_parameter_status("新演示会话，请重新读取参数。");
    replay_samples_.clear();
    unlock_->setChecked(false);
    freeze_->setChecked(false);
    scope_->Scope_SetSamples({});
    scope_->Scope_SetReplayFraction(std::nullopt);
    show_page(scenario == DemoScenario::Loader ? 7 : 1);
    refresh_workbench();
}

void MaintenanceWindow::connect_device()
{
    if (connection_->currentIndex() != 1 || device_reader_ || upgrade_process_ ||
        session_.Session_View().connected || sandbox_.Sandbox_SavePending())
    {
        return;
    }
    const auto helper = QCoreApplication::applicationDirPath() + "/device_reader.py";
    if (!QFileInfo::exists(python_path_->text()) || !QFileInfo::exists(sdk_path_->text()) ||
        !QFileInfo::exists(helper))
    {
        device_error_ = "缺少 Python 运行时、SDK DLL 或同目录 CAN 辅助脚本；连接未启动。";
        refresh_workbench();
        return;
    }
    device_error_.clear();
    const auto evidence_dir = QDir::cleanPath(
        QDir(QCoreApplication::applicationDirPath())
            .absoluteFilePath("../outputs/host-gui-" +
                              QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz")));
    if (!QDir().mkpath(evidence_dir))
    {
        device_error_ = "无法创建真实 GUI 证据目录；连接未启动。";
        refresh_workbench();
        return;
    }
    device_trace_path_ = QDir(evidence_dir).filePath("events.jsonl");
    append_device_trace("connect_intent",
                        {{"node", node_->value()},
                         {"expected_uid", "5700460010504e4831303320"},
                         {"sequence_start", device_sequence_start_->value()},
                         {"first_sequence", device_sequence_start_->value() + 1}});
    if (!device_error_.isEmpty())
    {
        refresh_workbench();
        return;
    }
    device_readback_.Device_Reset(quint8(node_->value()));
    device_output_.clear();
    device_error_.clear();
    device_ready_ = device_pending_ = false;
    device_motion_may_run_ = device_stop_ack_ = device_start_rejected_ = false;
    device_stop_deadline_ms_ = device_stop_retry_ms_ = 0;
    device_command_queued_.clear();
    device_control_result_.clear();
    device_sequence_ = uint16_t(device_sequence_start_->value());
    device_step_ = 0;
    device_connect_ms_ = device_discovery_ms_ = elapsed_.elapsed();
    device_feedback_ms_ = -1000;
    replay_samples_.clear();
    freeze_->setChecked(false);
    scope_->Scope_SetSamples({});
    parameter_snapshots_.clear();
    for (int row = 0; row < parameter_table_->rowCount(); ++row)
    {
        parameter_table_->item(row, 2)->setText("设备参数未接线");
        parameter_table_->item(row, 4)->setText("");
        parameter_table_->item(row, 4)->setFlags(parameter_table_->item(row, 4)->flags() &
                                                 ~Qt::ItemIsEditable);
    }
    revision_label_->setText("设备参数 revision：未读取；真实读写入口未接线");
    auto *reader = new QProcess(this);
    device_reader_ = reader;
    reader->setProgram(python_path_->text());
    reader->setArguments(
        {"-u", helper, "--dll", sdk_path_->text(), "--node", QString::number(node_->value())});
    reader->setWorkingDirectory(QCoreApplication::applicationDirPath());
    connect(reader,
            &QProcess::readyReadStandardOutput,
            this,
            [this]
            {
                consume_device_stdout();
            });
    connect(reader,
            &QProcess::errorOccurred,
            this,
            [this, reader]
            {
                if (device_reader_ == reader)
                {
                    device_error_ = "CAN 辅助进程启动或通信失败；检查 Python、SDK 和独占占用。";
                    disconnect_device();
                    refresh_workbench();
                }
            });
    connect(reader,
            &QProcess::finished,
            this,
            [this, reader]
            {
                if (device_reader_ == reader)
                {
                    if (device_error_.isEmpty())
                    {
                        device_error_ = "CAN 辅助进程已退出；设备读数失效。";
                    }
                    disconnect_device();
                    refresh_workbench();
                }
            });
    reader->start();
    refresh_workbench();
}

void MaintenanceWindow::append_device_trace(const QString &event, QJsonObject fields)
{
    if (device_trace_path_.isEmpty())
    {
        return;
    }
    fields["event"] = event;
    fields["utc"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs);
    QFile file(device_trace_path_);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Append) ||
        file.write(QJsonDocument(fields).toJson(QJsonDocument::Compact) + '\n') < 0)
    {
        device_error_ = "真实 GUI 证据写入失败；需人工核对本次操作。";
    }
}

void MaintenanceWindow::disconnect_device()
{
    auto *reader = device_reader_;
    if (!reader)
    {
        return;
    }
    if (device_motion_may_run_)
    {
        device_error_ = "控制状态未确认关相；辅助进程退出时尝试 STOP，需人工核对输出。";
    }
    append_device_trace(
        "disconnect", {{"motion_may_run", device_motion_may_run_}, {"stop_ack", device_stop_ack_}});
    device_reader_ = nullptr;
    device_ready_ = device_pending_ = false;
    device_command_queued_.clear();
    device_stop_ack_ = false;
    device_start_rejected_ = false;
    device_stop_deadline_ms_ = device_stop_retry_ms_ = 0;
    reader->closeWriteChannel();
    if (!reader->waitForFinished(500))
    {
        reader->kill();
        reader->waitForFinished(500);
    }
    reader->deleteLater();
    device_output_.clear();
    device_readback_.Device_Reset(quint8(node_->value()));
    device_bench_ack_->setChecked(false);
}

bool MaintenanceWindow::device_identity_matches() const
{
    const auto view = device_readback_.Device_View(elapsed_.elapsed());
    return device_reader_ && device_ready_ && !upgrade_process_ && view.identity_fresh &&
           view.uid == "5700460010504E4831303320" && view.role == 1 &&
           view.app_version == 0x00010300U && view.parameter_schema == 12;
}

bool MaintenanceWindow::device_safe_idle() const
{
    const auto view = device_readback_.Device_View(elapsed_.elapsed());
    return device_identity_matches() && view.feedback_fresh && view.motor_state == 5 &&
           view.control_mode == 0 && view.fault_bits == 0 && view.bus_voltage_v &&
           *view.bus_voltage_v > 0 && *view.bus_voltage_v <= 33.0 && view.bus_current_a &&
           std::abs(*view.bus_current_a) <= 3.0 && view.speed_rad_s &&
           std::abs(*view.speed_rad_s) < 0.5 && view.iq_a && std::abs(*view.iq_a) <= 6.0;
}

void MaintenanceWindow::queue_device_command(const QString &operation, int rpm)
{
    if (!device_reader_ || !device_ready_ || upgrade_process_)
    {
        return;
    }
    const auto view = device_readback_.Device_View(elapsed_.elapsed());
    if (operation == "stop")
    {
        device_command_queued_ = operation;
        device_command_rpm_ = 0;
        if (device_stop_deadline_ms_ == 0)
        {
            device_stop_deadline_ms_ = elapsed_.elapsed() + 1500;
        }
    }
    else if (device_command_queued_.isEmpty() && device_bench_ack_->isChecked() &&
             operation == "start_speed" && !device_motion_may_run_ && device_safe_idle())
    {
        device_command_queued_ = operation;
        device_command_rpm_ = 0;
    }
    else if (device_command_queued_.isEmpty() && operation == "set_speed" &&
             device_motion_may_run_ && device_identity_matches() && view.feedback_fresh &&
             view.motor_state == 6 && view.control_mode == 2 && view.fault_bits == 0 &&
             device_bench_ack_->isChecked() && rpm >= 0 && rpm <= 300)
    {
        device_command_queued_ = operation;
        device_command_rpm_ = rpm;
    }
    if (!device_command_queued_.isEmpty())
    {
        device_control_result_ = "控制请求待发送；回复受理不代表目标已应用。";
        append_device_trace("button",
                            {{"operation", device_command_queued_}, {"rpm", device_command_rpm_}});
    }
    refresh_workbench();
}

void MaintenanceWindow::issue_device_query()
{
    if (!device_reader_ || !device_ready_ || device_pending_ ||
        device_reader_->state() != QProcess::Running)
    {
        return;
    }
    const auto now_ms = elapsed_.elapsed();
    if (!device_command_queued_.isEmpty())
    {
        if (device_command_queued_ == "stop" && now_ms < device_stop_retry_ms_)
        {
            return;
        }
        const auto operation = device_command_queued_;
        const uint16_t type = operation == "start_speed" ? 117
                              : operation == "set_speed" ? 116
                                                         : 110;
        device_sequence_ = device_sequence_ == 65535 ? 1 : device_sequence_ + 1;
        const auto request = QJsonDocument(QJsonObject{{"op", operation},
                                                       {"sequence", int(device_sequence_)},
                                                       {"rpm", device_command_rpm_}})
                                 .toJson(QJsonDocument::Compact) +
                             '\n';
        if (device_reader_->write(request) != request.size())
        {
            device_error_ = "控制请求写入失败；需人工确认 STOP 与关相。";
            disconnect_device();
            return;
        }
        append_device_trace("command_sent",
                            {{"type", int(type)},
                             {"sequence", int(device_sequence_)},
                             {"rpm", device_command_rpm_}});
        device_command_queued_.clear();
        device_pending_ = true;
        device_pending_type_ = type;
        device_request_ms_ = now_ms;
        if (type == 117)
        {
            device_motion_may_run_ = true;
            device_stop_ack_ = false;
            device_start_rejected_ = false;
        }
        if (type == 110)
        {
            device_stop_ack_ = false;
        }
        return;
    }
    if (device_step_ >= 5 && now_ms - device_discovery_ms_ >= 3000)
    {
        device_step_ = 0;
        device_discovery_ms_ = now_ms;
    }
    if (device_step_ >= 5 && now_ms - device_feedback_ms_ < 200)
    {
        return;
    }
    const uint16_t type = device_step_ < 3    ? 1
                          : device_step_ == 3 ? 2
                          : device_step_ == 4 ? 3
                                              : 108;
    const uint16_t page = device_step_ < 3 ? uint16_t(device_step_) : 0;
    device_sequence_ = device_sequence_ == 65535 ? 1 : device_sequence_ + 1;
    if (!Window_BeginDeviceRead(type, device_sequence_, page))
    {
        device_error_ = "只读请求所有者拒绝新查询；连接已关闭。";
        disconnect_device();
        return;
    }
    const auto request = QJsonDocument(QJsonObject{{"type", int(type)},
                                                   {"sequence", int(device_sequence_)},
                                                   {"page", int(page)}})
                             .toJson(QJsonDocument::Compact) +
                         '\n';
    if (device_reader_->write(request) != request.size())
    {
        device_error_ = "CAN 辅助进程请求写入失败；连接已关闭。";
        disconnect_device();
        return;
    }
    device_pending_ = true;
    device_pending_type_ = type;
    device_request_ms_ = now_ms;
    if (device_step_ < 5)
    {
        ++device_step_;
    }
}

void MaintenanceWindow::consume_device_stdout()
{
    if (!device_reader_)
    {
        return;
    }
    device_output_ += device_reader_->readAllStandardOutput();
    if (device_output_.size() > 4096)
    {
        device_error_ = "CAN 辅助进程返回过长；连接已关闭。";
        disconnect_device();
        return;
    }
    while (device_reader_ && device_output_.contains('\n'))
    {
        const int end = device_output_.indexOf('\n');
        const auto line = device_output_.left(end);
        device_output_.remove(0, end + 1);
        const auto document = QJsonDocument::fromJson(line);
        if (!document.isObject())
        {
            device_error_ = "CAN 辅助进程输出非法 JSON；连接已关闭。";
            disconnect_device();
            return;
        }
        const auto message = document.object();
        const auto event = message.value("event").toString();
        if (event == "ready" && !device_ready_ && message.value("node").toInt() == node_->value())
        {
            device_ready_ = true;
            append_device_trace("port_ready", {{"node", node_->value()}});
            show_page(1);
        }
        else if (event == "frame" && device_pending_ &&
                 message.value("type").toInt() == device_pending_type_ &&
                 message.value("sequence").toInt() == device_sequence_)
        {
            const auto hex = message.value("data").toString().toLatin1();
            const auto frame = QByteArray::fromHex(hex);
            if (hex.size() > 128 || frame.toHex() != hex.toLower() ||
                !Window_ObserveDeviceFrame(uint32_t(message.value("can_id").toInt()), frame))
            {
                device_error_ = "只读回复未通过帧或请求关联校验；连接已关闭。";
                disconnect_device();
                return;
            }
            device_pending_ = false;
            append_device_trace("frame",
                                {{"type", int(device_pending_type_)},
                                 {"sequence", int(device_sequence_)},
                                 {"can_id", message.value("can_id")},
                                 {"data", message.value("data")}});
            if (device_pending_type_ == 108)
            {
                device_feedback_ms_ = elapsed_.elapsed();
                const auto view = device_readback_.Device_View(device_feedback_ms_);
                if (view.motor_state == 6)
                {
                    device_motion_may_run_ = true;
                    device_stop_ack_ = false;
                    device_start_rejected_ = false;
                }
                else if (view.motor_state == 5 && (device_stop_ack_ || device_start_rejected_))
                {
                    device_motion_may_run_ = false;
                    device_control_result_ =
                        device_start_rejected_ ? "启动被拒绝，108 确认 IDLE；未运行。"
                                               : "STOP 结果 0/0 且108确认 IDLE；仍需现场核对转子。";
                    device_start_rejected_ = false;
                }
                if (device_motion_may_run_ &&
                    (!view.feedback_fresh || !view.bus_voltage_v || *view.bus_voltage_v > 33.0 ||
                     !view.bus_current_a || std::abs(*view.bus_current_a) > 3.0 || !view.iq_a ||
                     std::abs(*view.iq_a) > 6.0 || !view.speed_rad_s ||
                     std::abs(*view.speed_rad_s) > 31.416 || view.fault_bits != 0))
                {
                    queue_device_command("stop");
                    device_control_result_ =
                        "108 越过台架上限或测量失效，已请求 STOP；需现场核对相电流。";
                }
            }
        }
        else if (event == "result" && device_pending_ && device_pending_type_ >= 110 &&
                 message.value("type").toInt() == device_pending_type_ &&
                 message.value("sequence").toInt() == device_sequence_)
        {
            const int result = message.value("result").toInt(-1);
            const int detail = message.value("detail").toInt(-1);
            if (result < 0 || detail < 0 || detail > 65535)
            {
                device_error_ = "控制回复字段非法；需人工确认输出。";
                disconnect_device();
                return;
            }
            device_control_result_ =
                QString("Type %1 回复 result=%2 detail=%3；以随后108实际状态为准。")
                    .arg(device_pending_type_)
                    .arg(result)
                    .arg(detail);
            append_device_trace("command_result",
                                {{"type", int(device_pending_type_)},
                                 {"sequence", int(device_sequence_)},
                                 {"result", result},
                                 {"detail", detail}});
            device_pending_ = false;
            if (device_pending_type_ == 117 && result != 0 && result != 1)
            {
                device_start_rejected_ = true;
                device_control_result_ += " 启动被拒绝；等待108 IDLE确认。";
                device_step_ = 5;
                device_discovery_ms_ = elapsed_.elapsed();
                device_feedback_ms_ = -1000;
            }
            if (device_pending_type_ == 110)
            {
                if (result == 1 && detail == 1 && elapsed_.elapsed() < device_stop_deadline_ms_)
                {
                    device_stop_ack_ = false;
                    device_command_queued_ = "stop";
                    device_command_rpm_ = 0;
                    device_stop_retry_ms_ = elapsed_.elapsed() + 50;
                    device_control_result_ = "STOP 已受理，正在查询最终关相确认。";
                }
                else if (result == 0 && detail == 0)
                {
                    device_stop_ack_ = true;
                    device_stop_deadline_ms_ = device_stop_retry_ms_ = 0;
                    device_step_ = 5;
                    device_discovery_ms_ = elapsed_.elapsed();
                    device_feedback_ms_ = -1000;
                    device_control_result_ = "STOP 结果 0/0；等待新的108 IDLE回读。";
                }
                else
                {
                    device_error_ = "STOP 未获最终关相确认；保持停机并人工检查。";
                    disconnect_device();
                    return;
                }
            }
        }
        else if (event == "error")
        {
            device_error_ = message.value("message").toString().left(180);
            append_device_trace("helper_error", {{"message", device_error_}});
            disconnect_device();
            return;
        }
        else
        {
            device_error_ = "CAN 辅助进程响应顺序或身份不符；连接已关闭。";
            disconnect_device();
            return;
        }
    }
    refresh_workbench();
}

QWidget *MaintenanceWindow::build_home_page()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 4, 0, 0);
    auto *row = new QHBoxLayout;
    for (const auto &section :
         {QStringList{"设备身份", "设备名称", "电机 ID", "电机类型", "节点地址", "UID / SN"},
          QStringList{
              "版本信息", "硬件版本", "软件版本", "Loader 版本", "参数结构版本", "活动参数版本"}})
    {
        auto *box = group(section.first());
        auto *form = new QFormLayout(box);
        form->setVerticalSpacing(16);
        for (const auto &key : section.mid(1))
        {
            auto *value = text_label("未识别", "identityValue");
            value->setTextInteractionFlags(Qt::TextSelectableByMouse);
            identity_[key] = value;
            form->addRow(key, value);
        }
        row->addWidget(box, 1);
    }
    layout->addLayout(row);
    auto *state = group("设备状态");
    auto *state_layout = new QVBoxLayout(state);
    auto *metrics = text_label("等待遥测", "homeTelemetry");
    state_layout->addWidget(metrics);
    auto *device_status =
        text_label("线路回读：未接入端口；无设备会话或功率操作权。", "deviceReadbackStatus");
    device_status->setWordWrap(true);
    state_layout->addWidget(device_status);
    state_layout->addWidget(text_label(
        "真实设备身份和版本保持未知；本地 target 的 schema 不是设备读回值。", "homeSourceNotice"));
    layout->addWidget(state);
    auto *flow = text_label("实机流程：六类业务视图尚无 CAN 线路；108 仅报告原八态。"
                            "CURRENT 单位 A，SPEED rad/s，POSITION rad，VQ V；Nm 模式不可用。"
                            "电感、磁链自动辨识及电机型号映射未实现，不能标记完成。",
                            "deviceFlowStatus");
    flow->setWordWrap(true);
    layout->addWidget(flow);
    auto *tasks = group("最近标定与辨识");
    auto *task_layout = new QVBoxLayout(tasks);
    task_summary_ = text_label("尚无任务", "taskSummary");
    task_summary_->setWordWrap(true);
    task_layout->addWidget(task_summary_);
    layout->addWidget(tasks);
    layout->addStretch();
    auto *start = action("进入调试工作台 →", "goWorkbench", "primary");
    layout->addWidget(start, 0, Qt::AlignLeft);
    connect(start,
            &QPushButton::clicked,
            this,
            [this]
            {
                show_page(2);
            });
    return page;
}

QWidget *MaintenanceWindow::build_workbench_page()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    auto *tools = new QHBoxLayout;
    stream_start_ = action("开始采集", "streamStart");
    auto *stream_stop = action("停止采集", "streamStop");
    record_start_ = action("开始记录", "recordStart");
    auto *record_stop = action("停止记录", "recordStop");
    freeze_ = new QCheckBox("冻结显示");
    freeze_->setObjectName("freezeScope");
    auto *window = new QComboBox;
    window->setObjectName("scopeWindow");
    window->addItems({"1 s", "5 s", "10 s", "30 s", "60 s"});
    window->setCurrentIndex(2);
    auto *export_csv = action("导出 CSV", "exportCapture");
    for (auto *item : {stream_start_, stream_stop, record_start_, record_stop})
    {
        tools->addWidget(item);
    }
    tools->addWidget(freeze_);
    tools->addWidget(window);
    tools->addStretch();
    tools->addWidget(export_csv);
    layout->addLayout(tools);
    telemetry_ = text_label("位置 — rad    转速 — rad/s    Iq — A    母线 — V    温度 — °C",
                            "telemetryStrip");
    layout->addWidget(telemetry_);
    auto *split = new QSplitter;
    split->setObjectName("workbenchSplitter");
    scope_ = new ScopeWidget;
    scope_->setObjectName("scope");
    split->addWidget(scope_);
    auto *right = new QWidget;
    auto *right_layout = new QVBoxLayout(right);
    right_layout->setContentsMargins(8, 0, 0, 0);
    right_layout->setSpacing(7);
    auto *motion = group("运动控制 · 演示");
    demo_motion_group_ = motion;
    auto *control = new QVBoxLayout(motion);
    control->setSpacing(7);
    unlock_ = new QCheckBox("解锁本机演示操作");
    unlock_->setObjectName("unlockWrites");
    control->addWidget(unlock_);
    motion_mode_ = new QComboBox;
    motion_mode_->setObjectName("motionMode");
    motion_mode_->addItems({"位置 / rad", "转速 / rad/s", "电流 Iq / A"});
    motion_target_ = new QDoubleSpinBox;
    motion_target_->setObjectName("motionTarget");
    motion_target_->setRange(-100, 100);
    motion_target_->setDecimals(3);
    motion_target_->setValue(3.14);
    motion_target_->setSuffix(" rad");
    control->addWidget(motion_mode_);
    control->addWidget(motion_target_);
    auto *enable_row = new QHBoxLayout;
    enable_ = action("使能", "enableMotor");
    auto *disable = action("失能", "disableMotor");
    enable_row->addWidget(enable_);
    enable_row->addWidget(disable);
    control->addLayout(enable_row);
    execute_ = action("执行演示运动", "executeMotion", "primary");
    control->addWidget(execute_);
    right_layout->addWidget(motion);
    auto *real_motion = group("实机速度控制 · 本轮架空台架");
    device_motion_group_ = real_motion;
    auto *real_controls = new QVBoxLayout(real_motion);
    device_bench_ack_ = new QCheckBox("架空、33 V / 母线 3 A / 相 6 A 已核对");
    device_bench_ack_->setObjectName("deviceBenchReady");
    real_controls->addWidget(device_bench_ack_);
    device_speed_rpm_ = new QSpinBox;
    device_speed_rpm_->setObjectName("deviceSpeedRpm");
    device_speed_rpm_->setRange(0, 300);
    device_speed_rpm_->setSuffix(" rpm");
    real_controls->addWidget(device_speed_rpm_);
    auto *real_actions = new QHBoxLayout;
    device_start_button_ = action("117 零目标启动", "deviceStartSpeed");
    device_speed_button_ = action("116 提交速度", "deviceSetSpeed", "primary");
    device_stop_button_ = action("110 STOP", "deviceStopSpeed");
    real_actions->addWidget(device_start_button_);
    real_actions->addWidget(device_stop_button_);
    real_controls->addLayout(real_actions);
    real_controls->addWidget(device_speed_button_);
    device_control_status_ = text_label("实机未连接；启动与目标均不可用。", "deviceControlStatus");
    device_control_status_->setWordWrap(true);
    real_controls->addWidget(device_control_status_);
    right_layout->addWidget(real_motion);
    auto *quick = group("常用参数");
    demo_quick_group_ = quick;
    auto *quick_layout = new QVBoxLayout(quick);
    quick_layout->setSpacing(7);
    quick_parameter_ = new QComboBox;
    quick_parameter_->setObjectName("quickParameter");
    const auto definitions = sandbox_.Sandbox_Definitions();
    for (int row : {1, 2, 5, 6, 7, 14, 15, 18, 19})
    {
        quick_parameter_->addItem(definitions[row].name + " / " + definitions[row].unit, row);
    }
    quick_active_ = text_label("活动值：未读取", "quickActive");
    quick_edit_ = new QLineEdit;
    quick_edit_->setObjectName("quickDraft");
    quick_edit_->setPlaceholderText("先读取参数");
    auto *quick_read = action("读取参数", "quickRead");
    quick_apply_ = action("应用选中草稿", "quickApply");
    quick_layout->addWidget(quick_parameter_);
    quick_layout->addWidget(quick_active_);
    quick_layout->addWidget(quick_edit_);
    auto *quick_actions = new QHBoxLayout;
    quick_actions->addWidget(quick_read);
    quick_actions->addWidget(quick_apply_);
    quick_layout->addLayout(quick_actions);
    quick_status_ = text_label("先读取参数", "quickStatus");
    quick_status_->setWordWrap(true);
    quick_layout->addWidget(quick_status_);
    right_layout->addWidget(quick, 1);
    right->setMinimumWidth(285);
    split->addWidget(right);
    split->setSizes({900, 335});
    split->setStretchFactor(0, 1);
    split->setStretchFactor(1, 0);
    layout->addWidget(split, 1);
    connect(unlock_,
            &QCheckBox::toggled,
            this,
            [this](bool value)
            {
                session_.Session_Unlock(value);
                refresh_workbench();
            });
    connect(enable_,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Enable(true);
                refresh_workbench();
            });
    connect(disable,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Enable(false);
                refresh_workbench();
            });
    connect(execute_,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Move(MotionMode(motion_mode_->currentIndex()),
                                      motion_target_->value());
                refresh_workbench();
            });
    connect(device_start_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                queue_device_command("start_speed");
            });
    connect(device_speed_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                queue_device_command("set_speed", device_speed_rpm_->value());
            });
    connect(device_stop_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                queue_device_command("stop");
            });
    connect(motion_mode_,
            &QComboBox::currentIndexChanged,
            this,
            [this](int index)
            {
                const double maximum = index == 0 ? 100 : index == 1 ? 10 : 3;
                motion_target_->setRange(-maximum, maximum);
                motion_target_->setSuffix(index == 0 ? " rad" : index == 1 ? " rad/s" : " A");
            });
    connect(stream_start_,
            &QPushButton::clicked,
            this,
            [this]
            {
                replay_samples_.clear();
                scope_->Scope_SetReplayFraction(std::nullopt);
                session_.Session_Stream(true);
                refresh_workbench();
            });
    connect(stream_stop,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Stream(false);
                refresh_workbench();
            });
    connect(record_start_,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Record(true);
                refresh_workbench();
            });
    connect(record_stop,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Record(false);
                refresh_workbench();
            });
    connect(freeze_, &QCheckBox::toggled, scope_, &ScopeWidget::Scope_SetFrozen);
    connect(window,
            &QComboBox::currentIndexChanged,
            this,
            [this, window]
            {
                scope_->Scope_SetWindow(window->currentText().split(' ').first().toDouble());
            });
    connect(export_csv,
            &QPushButton::clicked,
            this,
            [this]
            {
                export_capture();
            });
    connect(quick_read,
            &QPushButton::clicked,
            this,
            [this]
            {
                read_parameters();
                refresh_quick_parameter();
            });
    connect(quick_parameter_,
            &QComboBox::currentIndexChanged,
            this,
            [this]
            {
                refresh_quick_parameter();
            });
    connect(quick_edit_,
            &QLineEdit::textEdited,
            this,
            [this](const QString &value)
            {
                parameter_table_->item(quick_parameter_->currentData().toInt(), 4)->setText(value);
            });
    connect(quick_apply_,
            &QPushButton::clicked,
            this,
            [this]
            {
                parameter_table_->selectRow(quick_parameter_->currentData().toInt());
                write_parameter();
                refresh_workbench();
            });
    return page;
}

QWidget *MaintenanceWindow::build_task_page(bool identification)
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 5, 0, 0);
    auto *box = group(identification ? "参数辨识 · 演示" : "标定 · 演示");
    auto *form = new QVBoxLayout(box);
    form->setSpacing(15);
    auto &widgets = task_widgets_[identification ? 1 : 0];
    widgets.kind = new QComboBox;
    widgets.kind->setObjectName(identification ? "identificationKind" : "calibrationKind");
    const int first = identification ? 4 : 0, last = identification ? 8 : 4;
    for (int kind = first; kind < last; ++kind)
    {
        widgets.kind->addItem(Task_Name(TaskKind(kind)), kind);
    }
    if (identification)
    {
        auto *items = qobject_cast<QStandardItemModel *>(widgets.kind->model());
        items->item(int(TaskKind::Inductance) - first)->setEnabled(false);
        items->item(int(TaskKind::Inductance) - first)->setText("电感 · 实机算法缺失，暂不可用");
    }
    form->addWidget(widgets.kind);
    widgets.phase =
        text_label("尚无演示任务", identification ? "identificationPhase" : "calibrationPhase");
    widgets.progress = new QProgressBar;
    widgets.progress->setTextVisible(false);
    widgets.result =
        text_label("结果：未知", identification ? "identificationResult" : "calibrationResult");
    widgets.result->setWordWrap(true);
    form->addWidget(widgets.phase);
    form->addWidget(widgets.progress);
    form->addWidget(widgets.result);
    auto *actions = new QHBoxLayout;
    widgets.start = action(
        "开始演示任务", identification ? "startIdentification" : "startCalibration", "primary");
    widgets.cancel =
        action("取消任务", identification ? "cancelIdentification" : "cancelCalibration");
    widgets.apply =
        action("应用到演示模型", identification ? "applyIdentification" : "applyCalibration");
    widgets.export_result =
        action("导出候选", identification ? "exportIdentification" : "exportCalibration");
    for (auto *item : {widgets.start, widgets.cancel, widgets.apply, widgets.export_result})
    {
        actions->addWidget(item);
    }
    actions->addStretch();
    form->addLayout(actions);
    layout->addWidget(box);
    auto *conditions = group("执行条件与结果状态");
    auto *details = new QVBoxLayout(conditions);
    details->addWidget(
        text_label("开始要求：演示 APP 已连接、操作已解锁、无故障、模型已失能；任务互斥。"));
    auto *description = text_label(
        identification ? "电阻明确为相电阻；电感实机辨识未实现；惯量注明参照轴与负载；"
                         "摩擦保留电流域模型，不凭空换算转矩。"
                       : "角度传感器、电角度零位、机械参考零位与齿槽任务分别建模；电角度偏置不等于"
                         "机械位置归零。");
    description->setWordWrap(true);
    details->addWidget(description);
    details->addWidget(
        text_label("本页进度和结果仅来自离线演示。实机任务、质量、应用和掉电保存均未确认。"));
    if (identification)
    {
        auto *missing =
            text_label("实机电感/磁链辨识尚无算法与结果；Nm 力矩换算未实现，CURRENT 只显示 A。"
                       "电机型号尚无自动映射，不能据此套用参数。",
                       "deviceMissingCapabilities");
        missing->setWordWrap(true);
        details->addWidget(missing);
    }
    if (!identification)
    {
        auto *plan = text_label(
            "实机整计划未接线：父 ticket、子步骤 ticket、requested/completed_steps 和最终 "
            "COMMITTED 均未知；旧 IDLE 或单项 ACK 不算整计划完成。",
            "deviceCalibrationPlan");
        plan->setWordWrap(true);
        details->addWidget(plan);
    }
    layout->addWidget(conditions);
    layout->addStretch();
    connect(widgets.start,
            &QPushButton::clicked,
            this,
            [this, identification]
            {
                session_.Session_StartTask(
                    TaskKind(task_widgets_[identification ? 1 : 0].kind->currentData().toInt()));
                refresh_workbench();
            });
    connect(widgets.cancel,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_CancelTask();
                refresh_workbench();
            });
    connect(widgets.apply,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_ApplyTask();
                refresh_workbench();
            });
    connect(widgets.export_result,
            &QPushButton::clicked,
            this,
            [this]
            {
                export_task();
            });
    connect(widgets.kind,
            &QComboBox::currentIndexChanged,
            this,
            [this]
            {
                refresh_workbench();
            });
    return page;
}

QWidget *MaintenanceWindow::build_diagnostics_page()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 5, 0, 0);
    auto *tools = new QHBoxLayout;
    auto *clear = action("清除演示故障", "clearFault");
    auto *export_csv = action("导出完整 CSV", "exportRecordedCsv");
    auto *import_csv = action("加载演示 CSV 回放", "importCapture");
    tools->addWidget(clear);
    tools->addWidget(export_csv);
    tools->addWidget(import_csv);
    tools->addStretch();
    layout->addLayout(tools);
    record_status_ = text_label("尚无记录", "recordStatus");
    layout->addWidget(record_status_);
    replay_slider_ = new QSlider(Qt::Horizontal);
    replay_slider_->setObjectName("replayPosition");
    replay_slider_->setRange(0, 1000);
    replay_slider_->setValue(1000);
    layout->addWidget(text_label("回放位置（加载后在调试工作台查看三组波形）"));
    layout->addWidget(replay_slider_);
    events_ = new QTextEdit;
    events_->setObjectName("eventLog");
    events_->setReadOnly(true);
    layout->addWidget(events_, 1);
    layout->addWidget(
        text_label("记录保持 SI 单位及缺样间隙，CSV 标记演示来源；回放不具备设备操作权。"));
    connect(clear,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_ClearFault();
                refresh_workbench();
            });
    connect(export_csv,
            &QPushButton::clicked,
            this,
            [this]
            {
                export_capture();
            });
    connect(import_csv,
            &QPushButton::clicked,
            this,
            [this]
            {
                import_capture();
            });
    connect(replay_slider_,
            &QSlider::valueChanged,
            this,
            [this](int value)
            {
                if (!replay_samples_.empty())
                {
                    scope_->Scope_SetReplayFraction(value / 1000.0);
                }
            });
    return page;
}

bool MaintenanceWindow::parameter_writable() const
{
    const auto &state = session_.Session_View();
    return connection_->currentIndex() == 0 && session_.Session_CanWrite() && !state.enabled &&
           state.task.phase != TaskPhase::Running && !sandbox_.Sandbox_SavePending();
}

void MaintenanceWindow::refresh_quick_parameter()
{
    if (!parameter_table_ || !quick_parameter_)
    {
        return;
    }
    const int row = quick_parameter_->currentData().toInt();
    quick_active_->setText("活动值：" + parameter_table_->item(row, 2)->text());
    if (parameter_status_)
    {
        quick_status_->setText(parameter_status_->text());
    }
    const auto draft = parameter_table_->item(row, 4)->text();
    if (quick_edit_->text() != draft)
    {
        quick_edit_->setText(draft);
    }
    const bool editable =
        parameter_snapshots_.contains(row) && (parameter_snapshots_[row].access & 2);
    quick_apply_->setEnabled(parameter_writable() && editable);
    quick_edit_->setEnabled(connection_->currentIndex() == 0 && editable &&
                            session_.Session_View().scenario != DemoScenario::ReadOnly &&
                            session_.Session_View().scenario != DemoScenario::Loader);
}

void MaintenanceWindow::refresh_workbench()
{
    const auto &state = session_.Session_View();
    const auto device = device_readback_.Device_View(elapsed_.isValid() ? elapsed_.elapsed() : 0);
    global_status_->setToolTip(
        device_trace_path_.isEmpty() ? QString() : "本次真实 GUI 事件：" + device_trace_path_);
    const bool busy = state.task.phase == TaskPhase::Running || sandbox_.Sandbox_SavePending();
    global_status_->setText(
        state.connected ? QString("演示节点 %1  |  %2  |  故障 %3  |  %4")
                              .arg(state.node)
                              .arg(state.scenario == DemoScenario::Loader ? "Loader"
                                   : state.enabled                        ? "已使能"
                                                                          : "未使能")
                              .arg(state.fault ? "1（演示）" : "0（演示）")
                              .arg(state.unlocked ? "本机已解锁" : "本机未解锁")
        : replay_samples_.empty() ? "未连接  |  实机状态未知"
                                  : QString("演示 CSV 回放 · 源节点 %1 | 设备未连接")
                                        .arg(replay_samples_.back().demo_node));
    if (!state.connected && connection_->currentIndex() == 1 &&
        (device.identity_fresh || device.feedback_fresh))
    {
        global_status_->setText(QString("CAN FD %1 · 节点 %2 | 台架候选，非产品能力")
                                    .arg(device_ready_ ? "实机回读" : "离线线路回放")
                                    .arg(node_->value()));
    }
    else if (device_reader_)
    {
        global_status_->setText(device_ready_ ? "CAN FD 端口已打开 · 身份待读"
                                              : "CAN FD 正在打开端口 · 不发送功率命令");
    }
    connect_button_->setEnabled(connection_->currentIndex() != 2 && !state.connected &&
                                !device_reader_ && !upgrade_process_ &&
                                !sandbox_.Sandbox_SavePending());
    connect_button_->setText(connection_->currentIndex() == 0   ? "连接演示设备"
                             : connection_->currentIndex() == 1 ? "连接 CAN FD 台架"
                                                                : "设备后端待接入");
    scenario_->setEnabled(!state.connected && connection_->currentIndex() == 0);
    node_->setEnabled(!state.connected && !device_reader_);
    device_sequence_start_->setEnabled(connection_->currentIndex() == 1 && !device_reader_ &&
                                       !upgrade_process_);
    connection_->setEnabled(!device_reader_ && !upgrade_process_);
    python_path_->setEnabled(!device_reader_ && !upgrade_process_);
    sdk_path_->setEnabled(!device_reader_ && !upgrade_process_);
    disconnect_button_->setEnabled((state.connected || device_reader_) && !upgrade_process_);
    stop_->setEnabled(state.connected || device_reader_);
    stop_->setText(device_reader_ ? "110 STOP · 需核对关相" : "停止并失能");
    demo_motion_group_->setVisible(connection_->currentIndex() == 0);
    demo_quick_group_->setVisible(connection_->currentIndex() == 0);
    device_motion_group_->setVisible(connection_->currentIndex() == 1);
    navigation_[3]->setEnabled(connection_->currentIndex() == 0);
    navigation_[4]->setEnabled(connection_->currentIndex() == 0);
    enable_->setEnabled(session_.Session_CanWrite() && !busy);
    execute_->setEnabled(session_.Session_CanWrite() && state.enabled && !busy);
    const bool device_idle = device_safe_idle();
    const bool device_running = device_identity_matches() && device.feedback_fresh &&
                                device.motor_state == 6 && device.control_mode == 2 &&
                                device.fault_bits == 0;
    device_bench_ack_->setEnabled(device_reader_ && !upgrade_process_);
    device_speed_rpm_->setEnabled(device_running && device_bench_ack_->isChecked());
    device_start_button_->setEnabled(device_idle && !device_motion_may_run_ &&
                                     device_bench_ack_->isChecked() &&
                                     device_command_queued_.isEmpty());
    device_speed_button_->setEnabled(device_running && device_motion_may_run_ &&
                                     device_bench_ack_->isChecked() &&
                                     device_command_queued_.isEmpty());
    device_stop_button_->setEnabled(device_reader_ && device_ready_ && !upgrade_process_);
    device_control_status_->setText(
        device_reader_
            ? (device_identity_matches()
                   ? QString("UID/APP/schema已匹配台架；能力位0=未正式开放；%1%2")
                         .arg(device_control_result_.isEmpty() ? "等待显式操作。"
                                                               : device_control_result_)
                         .arg(device_motion_may_run_ ? " 运动可能仍活动，断开前须STOP并核对108。"
                                                     : "")
                   : "只读已连接；UID、APP 1.3.0 或 schema12 尚未齐备，功率入口关闭。")
            : "实机未连接；控制入口关闭。");
    const auto motion_index = motion_mode_->currentIndex();
    const double target_limit =
        motion_index == 0 ? 100.0
                          : sandbox_.Sandbox_Read(motion_index == 1 ? 1 : 2).value->raw / 1000.0;
    motion_target_->setRange(-target_limit, target_limit);
    stream_start_->setEnabled(state.connected && state.scenario != DemoScenario::Loader &&
                              !state.streaming);
    record_start_->setEnabled(state.streaming && !state.recording);
    read_button_->setEnabled(state.connected && state.scenario != DemoScenario::Loader);
    write_button_->setEnabled(parameter_writable() && !parameter_snapshots_.isEmpty());
    save_button_->setEnabled(parameter_writable() && !parameter_snapshots_.isEmpty());
    demo_button_->setEnabled(plan_ && plan_->valid && connection_->currentIndex() == 0 &&
                             !demo_timer_->isActive());
    if (upgrade_run_button_)
    {
        upgrade_run_button_->setEnabled(
            !upgrade_process_ && device_safe_idle() && device.loader_version == 0x00010000U &&
            device_stop_ack_ && !device_motion_may_run_ && !demo_timer_->isActive() &&
            !sandbox_.Sandbox_SavePending() && mode_ == UpgradeMode::App && plan_ && plan_->valid &&
            !plan_->synthetic);
    }
    const auto &s = replay_samples_.empty() ? state.sample : replay_samples_.back();
    const bool sampled = !session_.Session_Samples().empty() || !replay_samples_.empty();
    const auto metrics =
        sampled
            ? QString("位置 %1 rad   转速 %2 rad/s   Iq %3 A   母线 %4 V   温度 %5 °C   · 演示数据")
                  .arg(s.position_rad, 0, 'f', 3)
                  .arg(s.speed_rad_s, 0, 'f', 3)
                  .arg(s.iq_a, 0, 'f', 3)
                  .arg(s.voltage_v, 0, 'f', 1)
                  .arg(s.temperature_c, 0, 'f', 1)
            : "位置 — rad   转速 — rad/s   Iq — A   母线 — V   温度 — °C   · 等待演示采集";
    const auto source_metrics =
        replay_samples_.empty() ? metrics
                                : QString("CSV 回放 · 演示节点 %1 | ").arg(s.demo_node) + metrics;
    telemetry_->setText(source_metrics);
    findChild<QLabel *>("homeTelemetry")->setText(source_metrics);
    identity_["设备名称"]->setText(state.connected ? "离线演示设备" : "未识别");
    identity_["电机 ID"]->setText("设备未提供");
    identity_["电机类型"]->setText(state.connected ? "离线运动模型" : "设备未提供");
    identity_["节点地址"]->setText(state.connected ? QString::number(state.node) + "（演示）"
                                                   : "未连接");
    identity_["UID / SN"]->setText("设备未提供");
    for (const auto &key : {"硬件版本", "软件版本", "Loader 版本", "参数结构版本"})
    {
        identity_[key]->setText("设备未提供");
    }
    identity_["活动参数版本"]->setText(
        parameter_snapshots_.isEmpty()
            ? "未读取"
            : QString::number(parameter_snapshots_.first().revision) + "（演示 revision）");
    auto *device_status = findChild<QLabel *>("deviceReadbackStatus");
    if (!state.connected && connection_->currentIndex() == 1)
    {
        const auto number = [](const auto &value)
        {
            return value ? QString::number(*value) : QString("未知");
        };
        const auto version = [](const std::optional<uint32_t> &value)
        {
            return value && *value ? QString("%1.%2.%3")
                                         .arg(*value >> 16U)
                                         .arg((*value >> 8U) & 0xFFU)
                                         .arg(*value & 0xFFU)
                                   : QString("未知");
        };
        identity_["设备名称"]->setText(
            device.role ? (*device.role == 1 ? "APP（帧回读）" : "Loader（帧回读）") : "未识别");
        identity_["电机类型"]->setText(number(device.motor_type));
        identity_["节点地址"]->setText(QString("%1（线路目标）").arg(node_->value()));
        identity_["UID / SN"]->setText(
            device.uid.isEmpty()
                ? "未知"
                : device.uid + (device.sn.isEmpty() ? " / SN 未提供" : " / " + device.sn));
        identity_["硬件版本"]->setText(number(device.hardware_revision));
        identity_["软件版本"]->setText(version(device.app_version));
        identity_["Loader 版本"]->setText(version(device.loader_version));
        identity_["参数结构版本"]->setText(number(device.parameter_schema));
        findChild<QLabel *>("homeSourceNotice")
            ->setText(device_ready_ ? "上方来自明确打开的 CAN 端口；本地 target 不作设备身份。"
                                    : "上方为离线线路帧回放；没有打开适配器，不作实机结论。");
        findChild<QLabel *>("deviceFlowStatus")
            ->setText(QString("GET_CAPS supported_modes=%1；0 表示候选控制尚未产品开放。"
                              "本轮只接117速度启动/116速度目标/110 STOP；160～162真实维护未接。"
                              "齿槽/相电阻/摩擦项目已退役；108仍报原八态。")
                          .arg(device.supported_modes ? QString::number(*device.supported_modes)
                                                      : "未知/过期"));
        const auto wire =
            device.feedback_fresh
                ? QString("108 八态 %1 · 模式 %2 · 故障位 0x%3；位置 %4 rad，转速 %5 rad/s，Iq %6 "
                          "A，母线 %7 A")
                      .arg(number(device.motor_state),
                           number(device.control_mode),
                           QString::number(device.fault_bits.value_or(0), 16),
                           device.position_rad ? QString::number(*device.position_rad) : "无效",
                           device.speed_rad_s ? QString::number(*device.speed_rad_s) : "无效",
                           device.iq_a ? QString::number(*device.iq_a) : "无效",
                           device.bus_current_a ? QString::number(*device.bus_current_a) : "无效")
                : "108 反馈缺失或已过期；实机状态未知";
        findChild<QLabel *>("homeTelemetry")->setText("线路帧回读 · " + wire);
        telemetry_->setText("CAN FD 实机回读 · " + wire);
        device_status->setText(
            QString("身份 %1；boot_epoch %2；本次 STOP 回复 %3。不能推断持续关相或转子静止。")
                .arg(device.identity_fresh ? "5 s 内已校验" : "缺失/过期",
                     device.boot_epoch ? QString::number(*device.boot_epoch) : "无效/未知",
                     device_stop_ack_ ? "0/0，仍需108 IDLE与现场核对" : "无确认"));
        scope_->Scope_SetSamples({});
    }
    else
    {
        device_status->setText("线路回读：未接入端口；无设备会话或功率操作权。");
        findChild<QLabel *>("homeSourceNotice")
            ->setText("真实设备身份和版本保持未知；本地 target 的 schema 不是设备读回值。");
        findChild<QLabel *>("deviceFlowStatus")
            ->setText("实机流程：六类业务视图尚无 CAN 线路；108 仅报告原八态。"
                      "CURRENT 单位 A，SPEED rad/s，POSITION rad，VQ V；Nm 模式不可用。"
                      "电感、磁链自动辨识及电机型号映射未实现，不能标记完成。");
    }
    if (replay_samples_.empty() && (state.connected || connection_->currentIndex() != 1))
    {
        scope_->Scope_SetSamples(session_.Session_Samples());
    }
    const auto &task = state.task;
    task_summary_->setText(
        task.run_id ? QString("%1  ·  run %2  ·  %3\n%4\n演示模型应用：%5；设备保存：未知")
                          .arg(Task_Name(task.kind))
                          .arg(task.run_id)
                          .arg(task.stage)
                          .arg(task.result.isEmpty() ? "尚无候选结果" : task.result)
                          .arg(task.applied_to_demo ? "已标记" : "未应用")
                    : "尚无任务");
    for (auto &widgets : task_widgets_)
    {
        const bool current = widgets.kind->currentData().toInt() == int(task.kind) && task.run_id;
        widgets.phase->setText(current ? QString("run %1 · %2 · %3 s · %4%（演示）")
                                             .arg(task.run_id)
                                             .arg(task.stage)
                                             .arg(task.elapsed_s, 0, 'f', 1)
                                             .arg(task.progress_percent)
                                       : "此项尚无任务结果");
        widgets.progress->setValue(current ? task.progress_percent : 0);
        widgets.result->setText(current && !task.result.isEmpty()
                                    ? task.result +
                                          QString("\n演示模型应用：%1；设备保存：未知")
                                              .arg(task.applied_to_demo ? "已标记" : "未应用")
                                    : "结果：未知");
        widgets.start->setEnabled(session_.Session_CanWrite() && !state.enabled && !busy);
        widgets.kind->setEnabled(!busy);
        widgets.cancel->setEnabled(current && task.phase == TaskPhase::Running);
        widgets.apply->setEnabled(current && task.phase == TaskPhase::Completed &&
                                  parameter_writable() && !task.applied_to_demo);
        widgets.export_result->setEnabled(current && task.phase == TaskPhase::Completed);
    }
    record_status_->setText(connection_->currentIndex() == 1
                                ? "CAN FD 实机回读；此页演示记录不作为设备证据"
                                : QString("采集：%1  |  记录：%2  |  本次 %3 点  |  回放 %4 点")
                                      .arg(state.streaming ? "50 Hz 演示" : "已停止")
                                      .arg(state.recording ? "正在记录" : "未记录")
                                      .arg(session_.Session_Recording().size())
                                      .arg(replay_samples_.size()));
    const auto log = session_.Session_Events().join('\n');
    if (events_->toPlainText() != log)
    {
        events_->setPlainText(log);
    }
    footer_->setText(connection_->currentIndex() == 1
                         ? QString("%1   |   实机反馈 %2   |   控制 %3")
                               .arg(device_error_.isEmpty()
                                        ? (device_ready_ ? "CAN 已打开" : "CAN 未连接")
                                        : device_error_)
                               .arg(device.feedback_fresh ? "新鲜" : "缺失或过期")
                               .arg(device_motion_may_run_ ? "需确认 STOP" : "未启用")
                         : QString("%1   |   采集 %2   |   记录 %3 点   |   参数保存 %4")
                               .arg(state.message)
                               .arg(state.streaming ? "演示 50 Hz" : "停止")
                               .arg(session_.Session_Recording().size())
                               .arg(sandbox_.Sandbox_SavePending() ? "等待完成" : "无待办"));
    refresh_quick_parameter();
}

void MaintenanceWindow::Window_AdvanceDemo(double seconds)
{
    session_.Session_Tick(seconds);
    refresh_workbench();
}

bool MaintenanceWindow::Window_BeginDeviceRead(uint16_t type, uint16_t sequence, uint16_t page)
{
    if (connection_->currentIndex() != 1 || session_.Session_View().connected || type == 110)
    {
        return false;
    }
    return device_readback_.Device_Begin(type, sequence, page, elapsed_.elapsed());
}

bool MaintenanceWindow::Window_ObserveDeviceFrame(uint32_t can_id, const QByteArray &frame)
{
    if (connection_->currentIndex() != 1 || session_.Session_View().connected ||
        !device_readback_.Device_Accept(can_id, frame, elapsed_.elapsed()))
    {
        return false;
    }
    refresh_workbench();
    return true;
}

void MaintenanceWindow::export_capture()
{
    const auto path = QFileDialog::getSaveFileName(
        this, "导出演示完整记录到 outputs（新文件）", {}, "CSV (*.csv)");
    if (path.isEmpty())
    {
        return;
    }
    try
    {
        Capture_Write(path,
                      replay_samples_.empty() ? session_.Session_Recording() : replay_samples_);
        footer_->setText("演示 CSV 已导出；没有设备采集证明。");
    }
    catch (const std::exception &error)
    {
        footer_->setText(QString::fromUtf8(error.what()));
    }
}

void MaintenanceWindow::import_capture()
{
    const auto path =
        QFileDialog::getOpenFileName(this, "加载 YG_MC_HOST 演示 CSV", {}, "CSV (*.csv)");
    if (path.isEmpty())
    {
        return;
    }
    try
    {
        auto samples = Capture_Read(path);
        session_.Session_Disconnect();
        cancel_demo();
        unlock_->setChecked(false);
        freeze_->setChecked(false);
        replay_samples_ = std::move(samples);
        scope_->Scope_SetSamples(
            std::deque<ScopeSample>(replay_samples_.begin(), replay_samples_.end()));
        scope_->Scope_SetReplayFraction(1.0);
        replay_slider_->setValue(1000);
        show_page(2);
        refresh_workbench();
        footer_->setText("演示记录回放 · 设备未连接，运动入口禁用。");
    }
    catch (const std::exception &error)
    {
        footer_->setText(QString::fromUtf8(error.what()));
    }
}

void MaintenanceWindow::export_task()
{
    const auto &task = session_.Session_View().task;
    if (task.phase != TaskPhase::Completed)
    {
        return;
    }
    const auto path =
        QFileDialog::getSaveFileName(this, "导出演示候选到 outputs", {}, "JSON (*.json)");
    if (path.isEmpty())
    {
        return;
    }
    const auto bytes = QJsonDocument(QJsonObject{{"source", "offline_demo"},
                                                 {"task", Task_Name(task.kind)},
                                                 {"run_id", qint64(task.run_id)},
                                                 {"result", task.result},
                                                 {"applied_to_demo", task.applied_to_demo},
                                                 {"device_applied", false},
                                                 {"device_saved", false}})
                           .toJson();
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    {
        footer_->setText("演示候选导出失败");
        return;
    }
    footer_->setText("演示候选已导出；没有设备应用或保存结果。");
}
} // namespace yg::host
