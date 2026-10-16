/** 单页真实设备界面；设备 I/O 由既有辅助进程和升级客户端执行。 */
#include "maintenance_window.h"
#include "shared/targets/vector_mini_st_g431.h"
#include <QCheckBox>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QProgressBar>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSpinBox>
#include <QTextEdit>
#include <QTextDocument>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>

namespace yg::host
{
namespace
{
QLabel *value_label(const char *name)
{
    auto *label = new QLabel("未知");
    label->setObjectName(name);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    return label;
}

QPushButton *button(const QString &text, const char *name, const char *role = "secondary")
{
    auto *item = new QPushButton(text);
    item->setObjectName(name);
    item->setProperty("role", role);
    item->setMinimumHeight(36);
    return item;
}

QString version(const std::optional<uint32_t> &value)
{
    if (!value || !*value)
    {
        return "未知";
    }
    return QString("%1.%2.%3").arg(*value >> 16U).arg((*value >> 8U) & 0xFFU).arg(*value & 0xFFU);
}

template <typename T> QString number(const std::optional<T> &value, int precision = -1)
{
    if (!value)
    {
        return "未知";
    }
    return precision < 0 ? QString::number(*value) : QString::number(*value, 'f', precision);
}
} // namespace

MaintenanceWindow::MaintenanceWindow(QWidget *parent) : QMainWindow(parent)
{
    build_shell();
}

MaintenanceWindow::~MaintenanceWindow()
{
    disconnect_device();
}

void MaintenanceWindow::closeEvent(QCloseEvent *event)
{
    if (upgrade_process_)
    {
        upgrade_status_->setText("升级仍在运行；等待结果和证据写入。");
        event->ignore();
        return;
    }
    if (device_motion_may_run_ && device_reader_)
    {
        queue_device_command("stop");
        device_control_status_->setText("已请求 STOP；确认 108 IDLE 和输出关闭后再退出。");
        event->ignore();
        return;
    }
    event->accept();
}

void MaintenanceWindow::build_shell()
{
    setWindowTitle(QStringLiteral("YG_MC_HOST v" YG_MC_HOST_VERSION));
    resize(1000, 760);
    setMinimumSize(780, 600);
    auto *root = new QWidget;
    root->setObjectName("workspace");
    auto *layout = new QVBoxLayout(root);
    layout->setContentsMargins(16, 12, 16, 12);
    layout->setSpacing(12);

    auto *top = new QHBoxLayout;
    auto *brand = new QLabel(QStringLiteral("YG_MC_HOST v" YG_MC_HOST_VERSION));
    brand->setObjectName("brandCompact");
    top->addWidget(brand);
    global_status_ = new QLabel("未连接 · 读数未知");
    global_status_->setObjectName("globalStatus");
    top->addWidget(global_status_, 1);
    disconnect_button_ = button("断开", "disconnectDevice");
    stop_ = button("停止", "stopMotor", "danger");
    top->addWidget(disconnect_button_);
    top->addWidget(stop_);
    layout->addLayout(top);

    auto *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    auto *content = new QWidget;
    auto *sections = new QVBoxLayout(content);
    sections->setSpacing(10);

    auto *connection = new QGroupBox("连接");
    auto *connect_form = new QFormLayout(connection);
    node_ = new QSpinBox;
    node_->setObjectName("targetNode");
    node_->setRange(1, 254);
    node_->setValue(1);
    node_->setMaximumWidth(120);
    connect_form->addRow("设备节点", node_);
    auto *advanced_toggle = button("高级设置 ▸", "advancedSettings");
    advanced_toggle->setCheckable(true);
    auto *advanced_panel = new QWidget;
    advanced_panel->setObjectName("advancedPanel");
    auto *advanced_form = new QFormLayout(advanced_panel);
    advanced_form->setContentsMargins(0, 0, 0, 0);
    advanced_panel->setVisible(false);
    device_sequence_start_ = new QSpinBox;
    device_sequence_start_->setObjectName("benchSequenceStart");
    device_sequence_start_->setRange(0, 65535);
    device_sequence_start_->setToolTip(
        "0 自动使用本工具按 UID "
        "保存的控制序号；首次或其他主机历史须明确填写。非零起点不得早于已存记录，65535 后回到 1。");
    advanced_form->addRow("序号起点", device_sequence_start_);
    python_path_ = new QLineEdit;
    python_path_->setObjectName("pythonRuntimePath");
    python_path_->setPlaceholderText("选择 Python 3 可执行文件");
    auto *python_row = new QHBoxLayout;
    python_row->addWidget(python_path_, 1);
    auto *browse_python = button("浏览", "browsePython");
    python_row->addWidget(browse_python);
    advanced_form->addRow("Python", python_row);
    sdk_path_ = new QLineEdit;
    sdk_path_->setObjectName("canSdkPath");
    sdk_path_->setPlaceholderText("选择 ControlCANFD.dll");
    auto *sdk_row = new QHBoxLayout;
    sdk_row->addWidget(sdk_path_, 1);
    auto *browse_sdk = button("浏览", "browseSdk");
    sdk_row->addWidget(browse_sdk);
    advanced_form->addRow("SDK DLL", sdk_row);
    connect_button_ = button("连接", "connectDevice", "primary");
    auto *connect_row = new QHBoxLayout;
    connect_row->addWidget(connect_button_);
    connect_row->addWidget(advanced_toggle);
    connect_row->addStretch();
    connect_form->addRow({}, connect_row);
    connect_form->addRow({}, advanced_panel);
    sections->addWidget(connection);

    auto *readback = new QGroupBox("设备回读");
    auto *readback_grid = new QGridLayout(readback);
    uid_value_ = value_label("deviceUid");
    version_value_ = value_label("deviceAppVersion");
    loader_value_ = value_label("deviceLoaderVersion");
    mode_value_ = value_label("deviceMode");
    fault_value_ = value_label("deviceFault");
    speed_value_ = value_label("deviceSpeed");
    iq_value_ = value_label("deviceIq");
    bus_value_ = value_label("deviceBus");
    auto add_readback = [readback_grid](int row, int column, const QString &title, QLabel *value)
    {
        readback_grid->addWidget(new QLabel(title), row, column * 2);
        readback_grid->addWidget(value, row, column * 2 + 1);
    };
    add_readback(0, 0, "UID / SN", uid_value_);
    add_readback(1, 0, "APP 版本", version_value_);
    add_readback(2, 0, "Loader 版本", loader_value_);
    add_readback(3, 0, "状态 / 模式", mode_value_);
    add_readback(0, 1, "故障", fault_value_);
    add_readback(1, 1, "实际速度", speed_value_);
    add_readback(2, 1, "Iq", iq_value_);
    add_readback(3, 1, "母线", bus_value_);
    device_status_ = value_label("deviceReadbackStatus");
    readback_grid->addWidget(device_status_, 4, 0, 1, 4);
    sections->addWidget(readback);

    auto *control = new QGroupBox("速度控制");
    auto *control_layout = new QVBoxLayout(control);
    device_bench_ack_ =
        new QCheckBox("已核对上方 UID 为本次目标，且架空台架满足 33 V / 母线 3 A / 相 6 A 限值");
    device_bench_ack_->setObjectName("deviceBenchAcknowledged");
    control_layout->addWidget(device_bench_ack_);
    auto *control_row = new QHBoxLayout;
    device_start_button_ = button("启动（零速）", "deviceStartSpeed", "primary");
    device_speed_rpm_ = new QSpinBox;
    device_speed_rpm_->setObjectName("deviceSpeedRpm");
    device_speed_rpm_->setRange(0, 300);
    device_speed_rpm_->setSuffix(" rpm");
    device_speed_button_ = button("设置速度", "deviceSetSpeed");
    control_row->addWidget(device_start_button_);
    control_row->addWidget(device_speed_rpm_);
    control_row->addWidget(device_speed_button_);
    control_row->addStretch();
    control_layout->addLayout(control_row);
    device_control_status_ = value_label("deviceControlStatus");
    device_control_status_->setWordWrap(true);
    control_layout->addWidget(device_control_status_);
    sections->addWidget(control);

    auto *upgrade = new QGroupBox("APP 升级");
    auto *upgrade_layout = new QVBoxLayout(upgrade);
    auto *package_row = new QHBoxLayout;
    app_path_ = new QLineEdit;
    app_path_->setObjectName("appPath");
    app_path_->setPlaceholderText("选择 image.bin；同目录必须有 manifest.bin");
    package_row->addWidget(app_path_, 1);
    auto *choose_app = button("选择 APP 包", "chooseApp");
    package_row->addWidget(choose_app);
    upgrade_layout->addLayout(package_row);
    validation_ = value_label("validationStatus");
    validation_->setText("未选择 APP 包");
    validation_->setWordWrap(true);
    upgrade_layout->addWidget(validation_);
    auto *upgrade_row = new QHBoxLayout;
    auto *inspect = button("检查", "inspectFiles");
    upgrade_run_button_ = button("升级", "executeAppUpgrade", "primary");
    upgrade_row->addWidget(inspect);
    upgrade_row->addWidget(upgrade_run_button_);
    upgrade_row->addStretch();
    upgrade_layout->addLayout(upgrade_row);
    progress_ = new QProgressBar;
    progress_->setRange(0, 100);
    progress_->setValue(0);
    upgrade_layout->addWidget(progress_);
    upgrade_status_ = value_label("upgradeStatus");
    upgrade_status_->setText("未连接；升级需要配套 LoaderApp。");
    upgrade_status_->setWordWrap(true);
    upgrade_layout->addWidget(upgrade_status_);
    upgrade_log_ = new QTextEdit;
    upgrade_log_->setObjectName("upgradeLog");
    upgrade_log_->setReadOnly(true);
    upgrade_log_->document()->setMaximumBlockCount(100);
    upgrade_log_->setMaximumHeight(90);
    upgrade_layout->addWidget(upgrade_log_);
    sections->addWidget(upgrade);
    sections->addStretch();
    scroll->setWidget(content);
    layout->addWidget(scroll, 1);
    setCentralWidget(root);

    connect(advanced_toggle,
            &QPushButton::toggled,
            this,
            [advanced_toggle, advanced_panel](bool shown)
            {
                advanced_panel->setVisible(shown);
                advanced_toggle->setText(shown ? "高级设置 ▾" : "高级设置 ▸");
            });
    connect(browse_python,
            &QPushButton::clicked,
            this,
            [this]
            {
                const auto path = QFileDialog::getOpenFileName(this, "选择 Python 3");
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
    connect(choose_app,
            &QPushButton::clicked,
            this,
            [this]
            {
                const auto path =
                    QFileDialog::getOpenFileName(this, "选择 APP image.bin", {}, "BIN (*.bin)");
                if (!path.isEmpty())
                {
                    app_path_->setText(path);
                    inspect_files();
                }
            });
    connect(app_path_,
            &QLineEdit::textChanged,
            this,
            [this]
            {
                plan_.reset();
                validation_->setText("未校验");
                refresh_workbench();
            });
    connect(inspect,
            &QPushButton::clicked,
            this,
            [this]
            {
                inspect_files();
            });
    connect(upgrade_run_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                start_real_upgrade();
            });
    connect(connect_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                connect_device();
            });
    connect(disconnect_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                if (device_motion_may_run_)
                {
                    queue_device_command("stop");
                    return;
                }
                disconnect_device();
                refresh_workbench();
            });
    connect(stop_,
            &QPushButton::clicked,
            this,
            [this]
            {
                queue_device_command("stop");
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
    connect(device_bench_ack_,
            &QCheckBox::toggled,
            this,
            [this]
            {
                refresh_workbench();
            });
    connect(node_,
            &QSpinBox::valueChanged,
            this,
            [this]
            {
                device_readback_.Device_Reset(quint8(node_->value()));
                refresh_workbench();
            });
    device_readback_.Device_Reset(quint8(node_->value()));
    elapsed_.start();
    auto *timer = new QTimer(this);
    timer->setInterval(20);
    connect(timer,
            &QTimer::timeout,
            this,
            [this]
            {
                const auto now_ms = elapsed_.elapsed();
                if (device_reader_ && !device_ready_ && now_ms - device_connect_ms_ > 2000)
                {
                    device_error_ = "CAN 端口打开超时。";
                    disconnect_device();
                }
                if (device_pending_ && now_ms - device_request_ms_ > 250)
                {
                    device_error_ = "CAN 请求超时；须人工核对 STOP 与关相。";
                    disconnect_device();
                }
                if (device_reader_ && device_stop_deadline_ms_ > 0 &&
                    now_ms > device_stop_deadline_ms_)
                {
                    device_error_ = "STOP 最终确认超时；须人工核对关相。";
                    disconnect_device();
                }
                issue_device_query();
                refresh_workbench();
            });
    timer->start();
    refresh_workbench();
}

void MaintenanceWindow::inspect_files()
{
    plan_ = Upgrade_Inspect(app_path_->text());
    progress_->setValue(0);
    upgrade_log_->clear();
    validation_->setText(plan_->valid ? plan_->checks.join("\n") : "校验失败：" + plan_->error);
    upgrade_status_->setText(plan_->valid ? "本地包校验通过；设备兼容性仍需回读核对。"
                                          : "未准备升级");
    refresh_workbench();
}

void MaintenanceWindow::refresh_workbench()
{
    const auto view = device_readback_.Device_View(elapsed_.isValid() ? elapsed_.elapsed() : 0);
    const bool connected = device_reader_ && device_ready_;
    const bool readable = connected && view.identity_fresh;
    const bool feedback = connected && view.feedback_fresh;
    global_status_->setText(connected ? QString("CAN FD 已连接 · 节点 %1").arg(node_->value())
                            : device_reader_ ? "CAN FD 正在连接"
                                             : "未连接 · 读数未知");
    global_status_->setToolTip(device_trace_path_.isEmpty() ? QString()
                                                            : "事件：" + device_trace_path_);
    connect_button_->setEnabled(!device_reader_ && !upgrade_process_);
    disconnect_button_->setEnabled(device_reader_ && !upgrade_process_);
    stop_->setEnabled(connected && !upgrade_process_);
    node_->setEnabled(!device_reader_ && !upgrade_process_);
    device_sequence_start_->setEnabled(!device_reader_ && !upgrade_process_);
    python_path_->setEnabled(!device_reader_ && !upgrade_process_);
    sdk_path_->setEnabled(!device_reader_ && !upgrade_process_);
    device_bench_ack_->setEnabled(device_identity_matches() && feedback);
    const bool speed_supported = Device_SupportsSpeed(view);
    const bool running =
        speed_supported && device_identity_matches() && feedback && view.motor_state == 6 &&
        view.control_mode == 2 && view.fault_bits == 0 && view.bus_voltage_v &&
        *view.bus_voltage_v > 0 && *view.bus_voltage_v <= 33.0 && view.bus_current_a &&
        std::abs(*view.bus_current_a) <= 3.0 && view.iq_a && std::abs(*view.iq_a) <= 6.0 &&
        view.speed_rad_s && std::abs(*view.speed_rad_s) <= 31.416;
    device_start_button_->setEnabled(speed_supported && device_safe_idle() &&
                                     device_sequence_ready_ && device_sequence_storage_ok_ &&
                                     !device_motion_may_run_ && device_bench_ack_->isChecked() &&
                                     device_command_queued_.isEmpty());
    device_speed_rpm_->setEnabled(running && device_sequence_ready_ &&
                                  device_sequence_storage_ok_ && device_motion_may_run_ &&
                                  device_bench_ack_->isChecked());
    device_speed_button_->setEnabled(device_speed_rpm_->isEnabled() &&
                                     device_command_queued_.isEmpty());
    uid_value_->setText(readable ? view.uid + (view.sn.isEmpty() ? "" : " / " + view.sn) : "未知");
    version_value_->setText(readable ? version(view.app_version) : "未知");
    loader_value_->setText(readable ? version(view.loader_version) : "未知");
    mode_value_->setText(
        feedback ? QString("%1 / %2").arg(view.motor_state == 5   ? "IDLE"
                                          : view.motor_state == 6 ? "RUN"
                                                                  : number(view.motor_state),
                                          view.control_mode == 0   ? "无"
                                          : view.control_mode == 2 ? "速度"
                                                                   : number(view.control_mode))
                 : "未知");
    fault_value_->setText(feedback && view.fault_bits ? QString("0x%1").arg(*view.fault_bits, 0, 16)
                                                      : "未知");
    speed_value_->setText(
        feedback && view.speed_rad_s
            ? QString::number(*view.speed_rad_s * 60.0 / (2.0 * 3.141592653589793), 'f', 1) + " rpm"
            : "未知");
    iq_value_->setText(feedback ? number(view.iq_a, 2) + " A" : "未知");
    bus_value_->setText(feedback ? number(view.bus_voltage_v, 1) + " V / " +
                                       number(view.bus_current_a, 2) + " A"
                                 : "未知");
    const QString identity_status =
        !device_identity_matches() ? "产品/硬件/schema/ABI 不兼容；控制已禁用"
        : speed_supported          ? "身份与状态新鲜；请核对 UID 后确认台架"
                                   : "设备未声明SPEED或能力过期；可读取、STOP和配套升级";
    device_status_->setText(!device_error_.isEmpty() ? device_error_
                            : !connected             ? "未连接"
                            : readable && feedback   ? identity_status
                                                     : "身份或状态缺失/过期");
    device_control_status_->setText(
        connected ? (device_control_result_.isEmpty() ? "等待显式操作" : device_control_result_)
                  : "未连接；运动状态未知");
    upgrade_run_button_->setEnabled(!upgrade_process_ && device_safe_idle() &&
                                    view.loader_version == TARGET_LOADER_VERSION &&
                                    device_stop_ack_ && !device_motion_may_run_ &&
                                    device_bench_ack_->isChecked() && plan_ && plan_->valid);
    if (!upgrade_process_ && view.loader_version && *view.loader_version != TARGET_LOADER_VERSION)
    {
        upgrade_status_->setText("需先安装配套 LoaderApp，不能升级。");
    }
}

void MaintenanceWindow::start_real_upgrade()
{
    plan_ = Upgrade_Inspect(app_path_->text());
    if (!plan_->valid)
    {
        validation_->setText("校验失败：" + plan_->error);
        upgrade_status_->setText("APP 包已变化或损坏；未打开升级端口。");
        refresh_workbench();
        return;
    }
    const auto view = device_readback_.Device_View(elapsed_.elapsed());
    if (upgrade_process_ || !device_safe_idle() || view.loader_version != TARGET_LOADER_VERSION ||
        !device_stop_ack_ || device_motion_may_run_ || !device_bench_ack_->isChecked() || !plan_ ||
        !plan_->valid)
    {
        upgrade_status_->setText(
            "升级门禁未满足：配套Loader、STOP/IDLE、零速与台架限值、独占和真实包均须核对。");
        return;
    }
    const auto expected_plan = *plan_;
    const auto app_dir = QCoreApplication::applicationDirPath();
    const auto client = app_dir + "/upgrade_client.py";
    if (!QFileInfo::exists(client) || !QFileInfo::exists(app_path_->text()))
    {
        upgrade_status_->setText("缺少同目录升级客户端或已校验 image.bin；未打开设备。");
        return;
    }
    upgrade_expected_uid_ = view.uid;
    upgrade_output_dir_ = QDir::cleanPath(QDir(app_dir).absoluteFilePath(
        "../outputs/host-upgrade-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz")));
    const auto python = python_path_->text();
    const auto sdk = sdk_path_->text();
    const auto node = node_->value();
    const auto package_dir = QFileInfo(app_path_->text()).absolutePath();
    append_device_trace("upgrade_click",
                        {{"package", package_dir},
                         {"output", upgrade_output_dir_},
                         {"uid", upgrade_expected_uid_}});
    disconnect_device();
    auto *process = new QProcess(this);
    upgrade_process_ = process;
    upgrade_output_.clear();
    process->setProcessChannelMode(QProcess::MergedChannels);
    process->setProgram(python);
    process->setArguments({"-u",
                           client,
                           "upgrade",
                           "--package",
                           package_dir,
                           "--sdk",
                           sdk,
                           "--node",
                           QString::number(node),
                           "--uid",
                           upgrade_expected_uid_,
                           "--output",
                           upgrade_output_dir_,
                           "--exclusive-bench"});
    process->setWorkingDirectory(app_dir);
    connect(process,
            &QProcess::readyReadStandardOutput,
            this,
            [this]
            {
                consume_upgrade_stdout();
            });
    connect(process,
            &QProcess::errorOccurred,
            this,
            [this, process]
            {
                if (upgrade_process_ == process && process->error() == QProcess::FailedToStart)
                {
                    upgrade_process_ = nullptr;
                    upgrade_status_->setText("升级客户端未启动；设备结果未知。");
                    process->deleteLater();
                    refresh_workbench();
                }
            });
    connect(process,
            &QProcess::finished,
            this,
            [this, process, expected_plan](int exit_code)
            {
                if (upgrade_process_ != process)
                {
                    return;
                }
                consume_upgrade_stdout();
                QFile summary(QDir(upgrade_output_dir_).filePath("summary.json"));
                const bool opened = summary.open(QIODevice::ReadOnly);
                const auto report =
                    opened ? QJsonDocument::fromJson(summary.readAll()).object() : QJsonObject{};
                const bool confirmed =
                    exit_code == 0 &&
                    Upgrade_ResultMatches(expected_plan, upgrade_expected_uid_, report);
                append_device_trace("upgrade_finish",
                                    {{"exit_code", exit_code},
                                     {"protocol_confirmed", confirmed},
                                     {"summary", upgrade_output_dir_ + "/summary.json"}});
                upgrade_status_->setText(
                    confirmed ? "APP 协议升级和激活确认完成；仍需 T 独立核对物理状态。"
                              : "APP 升级失败或确认不完整；保持停机，人工检查 Loader/输出。");
                upgrade_log_->append("证据：" + upgrade_output_dir_ + "/summary.json");
                if (confirmed)
                {
                    progress_->setValue(100);
                }
                upgrade_process_ = nullptr;
                process->deleteLater();
                refresh_workbench();
            });
    upgrade_status_->setText("APP 升级客户端已启动；不得关闭窗口或并发使用 CAN/J-Link。");
    upgrade_log_->append("真实 APP 升级 · 目标 UID " + upgrade_expected_uid_);
    progress_->setValue(0);
    process->start();
    refresh_workbench();
}

void MaintenanceWindow::consume_upgrade_stdout()
{
    if (!upgrade_process_)
    {
        return;
    }
    upgrade_output_ += upgrade_process_->readAllStandardOutput();
    while (upgrade_output_.contains('\n'))
    {
        const int end = upgrade_output_.indexOf('\n');
        const auto line = QString::fromUtf8(upgrade_output_.left(end)).trimmed();
        upgrade_output_.remove(0, end + 1);
        if (line.isEmpty())
        {
            continue;
        }
        upgrade_log_->append(line.left(180));
        const auto match = QRegularExpression("下载 (\\d+)%").match(line);
        if (match.hasMatch())
        {
            progress_->setValue(match.captured(1).toInt());
        }
    }
    if (upgrade_output_.size() > 4096)
    {
        upgrade_output_ = upgrade_output_.right(4096);
    }
}
} // namespace yg::host
