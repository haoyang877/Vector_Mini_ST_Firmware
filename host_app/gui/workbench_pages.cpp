/** 工作区只组合会话与参数服务；维护功能是其中一页，不替换完整工作台。 */
#include "maintenance_window.h"
#include "scope_widget.h"
#include "services/capture_csv.h"
#include <QButtonGroup>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>

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
    setWindowTitle("YG_MC_HOST · 电机调试工作台");
    resize(1366, 768);
    setMinimumSize(1080, 680);
    auto *root = new QWidget;
    root->setObjectName("workspace");
    auto *layout = new QVBoxLayout(root);
    layout->setContentsMargins(16, 12, 16, 10);
    layout->setSpacing(9);
    auto *top = new QHBoxLayout;
    top->addWidget(text_label("YG_MC_HOST", "brandCompact"));
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
    auto *notice = text_label("演示数据与真实设备隔离 · CAN FD / SWD 设备后端待接入", "notice");
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
                session_.Session_Disconnect();
                cancel_demo();
                unlock_->setChecked(false);
                refresh_workbench();
            });
    connect(stop_,
            &QPushButton::clicked,
            this,
            [this]
            {
                session_.Session_Stop();
                refresh_workbench();
            });
    connect(connection_,
            &QComboBox::currentIndexChanged,
            this,
            [this]
            {
                session_.Session_Disconnect();
                cancel_demo();
                unlock_->setChecked(false);
                refresh_workbench();
            });
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
    connection_->addItems({"离线演示 · 不连接设备", "CAN FD · 待接入", "SWD · 待接入"});
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
    form->addWidget(text_label("演示场景"));
    form->addWidget(scenario_);
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
                connect_demo();
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
    state_layout->addWidget(
        text_label("真实设备身份和版本保持未知；本地 target 的 schema 不是设备读回值。"));
    layout->addWidget(state);
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
    auto *quick = group("常用参数");
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
        identification ? "电阻明确为相电阻；电感分别展示 "
                         "Ld/Lq；惯量注明参照轴与负载；摩擦保留电流域模型，不凭空换算转矩。"
                       : "角度传感器、电角度零位、机械参考零位与齿槽任务分别建模；电角度偏置不等于"
                         "机械位置归零。");
    description->setWordWrap(true);
    details->addWidget(description);
    details->addWidget(
        text_label("本页进度和结果仅来自离线演示。实机任务、质量、应用和掉电保存均未确认。"));
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
    quick_edit_->setEnabled(editable &&
                            session_.Session_View().scenario != DemoScenario::ReadOnly &&
                            session_.Session_View().scenario != DemoScenario::Loader);
}

void MaintenanceWindow::refresh_workbench()
{
    const auto &state = session_.Session_View();
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
    connect_button_->setEnabled(connection_->currentIndex() == 0 && !state.connected &&
                                !sandbox_.Sandbox_SavePending());
    connect_button_->setText(connection_->currentIndex() == 0 ? "连接演示设备" : "设备后端待接入");
    scenario_->setEnabled(!state.connected && connection_->currentIndex() == 0);
    node_->setEnabled(!state.connected);
    disconnect_button_->setEnabled(state.connected);
    stop_->setEnabled(state.connected);
    enable_->setEnabled(session_.Session_CanWrite() && !busy);
    execute_->setEnabled(session_.Session_CanWrite() && state.enabled && !busy);
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
    if (replay_samples_.empty())
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
    record_status_->setText(QString("采集：%1  |  记录：%2  |  本次 %3 点  |  回放 %4 点")
                                .arg(state.streaming ? "50 Hz 演示" : "已停止")
                                .arg(state.recording ? "正在记录" : "未记录")
                                .arg(session_.Session_Recording().size())
                                .arg(replay_samples_.size()));
    const auto log = session_.Session_Events().join('\n');
    if (events_->toPlainText() != log)
    {
        events_->setPlainText(log);
    }
    footer_->setText(QString("%1   |   采集 %2   |   记录 %3 点   |   参数保存 %4")
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
