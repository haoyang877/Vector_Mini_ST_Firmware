/** 原生页面调用离线业务服务；任何按钮均不加载驱动、连接 CAN/SWD 或写设备。 */
#include "maintenance_window.h"
#include <QButtonGroup>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSpinBox>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <cmath>
#include <limits>

namespace yg::host
{
namespace
{
QLabel *label(const QString &text, const char *style, QWidget *parent = nullptr)
{
    auto *item = new QLabel(text, parent);
    item->setObjectName(style);
    return item;
}

QFrame *card(QVBoxLayout *layout, const QString &title)
{
    auto *frame = new QFrame;
    frame->setObjectName("card");
    auto *content = new QVBoxLayout(frame);
    content->setContentsMargins(20, 17, 20, 19);
    content->setSpacing(13);
    content->addWidget(label(title, "cardTitle"));
    layout->addWidget(frame);
    return frame;
}

QPushButton *button(const QString &text, const char *name, const char *role = "secondary")
{
    auto *item = new QPushButton(text);
    item->setObjectName(name);
    item->setProperty("role", role);
    item->setCursor(Qt::PointingHandCursor);
    item->setMinimumHeight(36);
    return item;
}

double scalar(const ParameterValue &value)
{
    return value.value_type == 2
               ? double(value.raw)
               : double(value.raw <= 0x7fffffff ? qint64(value.raw)
                                                : qint64(value.raw) - 0x100000000LL);
}

QString reason(const ParameterReply &reply)
{
    if (reply.detail == 3)
    {
        return "revision 冲突，请重新读取后再编辑";
    }
    if (reply.detail == 2)
    {
        return "该参数只读或由标定所有者管理";
    }
    if (reply.result == 6)
    {
        return "保存作业忙，请等待完成";
    }
    return QString("请求被拒绝：result=%1，detail=%2").arg(reply.result).arg(reply.detail);
}
} // namespace

MaintenanceWindow::MaintenanceWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle("YG_MC_HOST · 升级与参数");
    resize(1380, 940);
    setMinimumSize(1120, 820);
    auto *root = new QWidget;
    root->setObjectName("workspace");
    auto *outer = new QHBoxLayout(root);
    outer->setContentsMargins(0, 0, 0, 0);
    outer->setSpacing(0);
    auto *sidebar = new QFrame;
    sidebar->setObjectName("sidebar");
    sidebar->setFixedWidth(215);
    auto *navigation = new QVBoxLayout(sidebar);
    navigation->setContentsMargins(19, 30, 19, 25);
    navigation->setSpacing(10);
    navigation->addWidget(label("YG_MC_HOST", "brand"));
    navigation->addWidget(label("MOTOR DRIVE WORKSPACE", "sidebarHint"));
    navigation->addSpacing(35);
    auto *upgrade_nav = button("固件升级", "upgradeNav", "nav");
    auto *parameter_nav = button("参数管理", "parameterNav", "nav");
    auto *group = new QButtonGroup(this);
    for (auto *item : {upgrade_nav, parameter_nav})
    {
        item->setCheckable(true);
        group->addButton(item);
        navigation->addWidget(item);
    }
    upgrade_nav->setChecked(true);
    navigation->addStretch();
    navigation->addWidget(label("目标配置\nSTM32G431CBUx\nCAN FD · 1M / 5M", "sidebarHint"));
    navigation->addSpacing(18);
    navigation->addWidget(label("SYSTEM REFACTOR\n离线开发版", "sidebarHint"));
    outer->addWidget(sidebar);
    auto *workspace = new QWidget;
    auto *layout = new QVBoxLayout(workspace);
    layout->setContentsMargins(29, 24, 29, 23);
    layout->setSpacing(15);
    auto *heading = new QHBoxLayout;
    auto *heading_text = new QVBoxLayout;
    heading_text->addWidget(label("DEVICE MAINTENANCE", "eyebrow"));
    title_ = label("固件升级", "pageTitle");
    subtitle_ = label("选择升级范围，先校验镜像，再准备安装任务。", "subtitle");
    heading_text->addWidget(title_);
    heading_text->addWidget(subtitle_);
    heading->addLayout(heading_text);
    heading->addStretch();
    heading->addWidget(label("离线演示", "badge"));
    layout->addLayout(heading);
    auto *device = card(layout, "设备与目标配置");
    auto *device_row = new QHBoxLayout;
    device_row->addWidget(label("连接方式", "subtitle"));
    connection_ = new QComboBox;
    connection_->setObjectName("connectionMode");
    connection_->addItems({"离线演示 · 不连接设备", "CAN FD · 待接线", "SWD · 待接线"});
    connection_->setMinimumWidth(230);
    device_row->addWidget(connection_);
    device_row->addSpacing(15);
    device_row->addWidget(label("目标节点", "subtitle"));
    auto *node = new QSpinBox;
    node->setObjectName("targetNode");
    node->setRange(1, 254);
    node->setValue(3);
    node->setToolTip("实际接线时必须按设备配置明确选择；当前不会发送任何帧。");
    device_row->addWidget(node);
    device_row->addStretch();
    device_row->addWidget(label("实际设备：未连接", "subtitle"));
    static_cast<QVBoxLayout *>(device->layout())->addLayout(device_row);
    const auto target = Upgrade_Target();
    auto constants = target["constants"].toObject();
    static_cast<QVBoxLayout *>(device->layout())
        ->addWidget(
            label(QString("目标 %1    |    layout %2    |    schema %3    |    参数区受保护")
                      .arg(target["device"].toString(),
                           constants["TARGET_LAYOUT"].toString().remove('U'),
                           constants["TARGET_SCHEMA"].toString().remove('U')),
                  "subtitle"));
    auto *notice =
        label("当前为离线开发版。镜像校验与任务导出可用；演示读写仅改变示例数据。", "notice");
    notice->setWordWrap(true);
    layout->addWidget(notice);
    pages_ = new QStackedWidget;
    pages_->addWidget(build_upgrade_page());
    pages_->addWidget(build_parameter_page());
    layout->addWidget(pages_, 1);
    outer->addWidget(workspace, 1);
    setCentralWidget(root);
    connect(upgrade_nav,
            &QPushButton::clicked,
            this,
            [this]
            {
                pages_->setCurrentIndex(0);
                title_->setText("固件升级");
                subtitle_->setText("选择升级范围，先校验镜像，再准备安装任务。");
            });
    connect(parameter_nav,
            &QPushButton::clicked,
            this,
            [this]
            {
                pages_->setCurrentIndex(1);
                title_->setText("参数管理");
                subtitle_->setText("读取活动参数，核对修订号，写入与掉电保存分别确认。");
            });
    connect(connection_,
            &QComboBox::currentIndexChanged,
            this,
            [this](int index)
            {
                cancel_demo();
                const bool offline = index == 0;
                const bool writable =
                    offline && !sandbox_.Sandbox_SavePending() && !parameter_snapshots_.isEmpty();
                read_button_->setEnabled(offline);
                write_button_->setEnabled(writable);
                save_button_->setEnabled(writable);
                demo_button_->setEnabled(offline && plan_ && plan_->valid);
                upgrade_status_->setText(offline ? "仅演示安装步骤，不操作设备"
                                                 : "设备执行尚未接线，可先校验文件和导出任务");
                set_parameter_status(offline ? "离线示例已选中；请读取参数"
                                             : "参数设备传输尚未接线，没有设备读写结果");
            });
}

QWidget *MaintenanceWindow::build_upgrade_page()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(14);
    layout->setSizeConstraint(QLayout::SetMinimumSize);
    auto *range = card(layout, "01  选择升级范围");
    auto *modes = new QHBoxLayout;
    auto *group = new QButtonGroup(this);
    const QStringList names{
        "Loader\n离线安装 · 保留 APP", "APP\nCAN FD · 保留 Loader", "Loader + APP\n配套离线安装"};
    for (int i = 0; i < names.size(); ++i)
    {
        auto *item = button(names[i], qPrintable(QString("mode%1").arg(i)), "mode");
        item->setCheckable(true);
        group->addButton(item);
        mode_buttons_.append(item);
        modes->addWidget(item, 1);
        connect(item,
                &QPushButton::clicked,
                this,
                [this, i]
                {
                    select_mode(UpgradeMode(i));
                });
    }
    mode_buttons_[1]->setChecked(true);
    static_cast<QVBoxLayout *>(range->layout())->addLayout(modes);
    route_ = label("APP 通过 CAN FD 升级；Loader 区保持保护。", "subtitle");
    route_->setWordWrap(true);
    static_cast<QVBoxLayout *>(range->layout())->addWidget(route_);
    auto *files = card(layout, "02  选择镜像与本地校验");
    auto *fields = static_cast<QVBoxLayout *>(files->layout());
    for (int i = 0; i < 2; ++i)
    {
        auto *row = new QHBoxLayout;
        auto *name = label(i == 0 ? "Loader BIN" : "APP image.bin", "subtitle");
        name->setFixedWidth(115);
        row->addWidget(name);
        auto *input = new QLineEdit;
        input->setObjectName(i == 0 ? "loaderPath" : "appPath");
        input->setPlaceholderText(i == 0 ? "选择 Loader 链接 BIN"
                                         : "选择打包 image.bin，同目录需有 manifest.bin");
        row->addWidget(input, 1);
        auto *choose = button("选择文件", i == 0 ? "chooseLoader" : "chooseApp");
        row->addWidget(choose);
        fields->addLayout(row);
        if (i == 0)
        {
            loader_path_ = input;
        }
        else
        {
            app_path_ = input;
        }
        connect(input,
                &QLineEdit::textEdited,
                this,
                [this]
                {
                    synthetic_ = false;
                    invalidate_plan();
                });
        connect(choose,
                &QPushButton::clicked,
                this,
                [this, input]
                {
                    const auto path =
                        QFileDialog::getOpenFileName(this, "选择 BIN 镜像", {}, "固件镜像 (*.bin)");
                    if (!path.isEmpty())
                    {
                        synthetic_ = false;
                        input->setText(path);
                        invalidate_plan();
                    }
                });
    }
    validation_ = label("尚未校验 · APP 必须使用当前 loader-image-v2-crc32 包。", "subtitle");
    validation_->setObjectName("validationStatus");
    validation_->setWordWrap(true);
    validation_->setMinimumHeight(45);
    fields->addWidget(validation_);
    auto *actions = new QHBoxLayout;
    auto *samples = button("载入演示样本", "loadSamples");
    auto *inspect = button("校验文件", "inspectFiles", "primary");
    export_job_button_ = button("导出升级任务", "exportJob");
    export_job_button_->setEnabled(false);
    actions->addWidget(samples);
    actions->addStretch();
    actions->addWidget(inspect);
    actions->addWidget(export_job_button_);
    fields->addLayout(actions);
    connect(samples,
            &QPushButton::clicked,
            this,
            [this]
            {
                load_samples();
            });
    connect(inspect,
            &QPushButton::clicked,
            this,
            [this]
            {
                inspect_files();
            });
    connect(export_job_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                export_job();
            });
    auto *progress_card = card(layout, "03  安装步骤与结果");
    auto *result = static_cast<QVBoxLayout *>(progress_card->layout());
    upgrade_status_ = label("仅演示安装步骤，不操作设备", "subtitle");
    upgrade_status_->setObjectName("upgradeStatus");
    result->addWidget(upgrade_status_);
    progress_ = new QProgressBar;
    progress_->setObjectName("upgradeProgress");
    progress_->setTextVisible(false);
    progress_->setValue(0);
    result->addWidget(progress_);
    upgrade_log_ = new QTextEdit;
    upgrade_log_->setObjectName("upgradeLog");
    upgrade_log_->setReadOnly(true);
    upgrade_log_->setMinimumHeight(95);
    upgrade_log_->setMaximumHeight(180);
    result->addWidget(upgrade_log_, 1);
    auto *run = new QHBoxLayout;
    run->addWidget(label("实机安装结果：未知", "subtitle"));
    run->addStretch();
    cancel_button_ = button("取消演示", "cancelDemo");
    cancel_button_->setEnabled(false);
    demo_button_ = button("演示升级流程", "startDemo", "primary");
    demo_button_->setEnabled(false);
    run->addWidget(cancel_button_);
    run->addWidget(demo_button_);
    result->addLayout(run);
    connect(demo_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                start_demo();
            });
    connect(cancel_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                cancel_demo();
            });
    demo_timer_ = new QTimer(this);
    demo_timer_->setInterval(220);
    connect(demo_timer_,
            &QTimer::timeout,
            this,
            [this]
            {
                if (!plan_ || demo_step_ >= plan_->steps.size())
                {
                    cancel_demo();
                    return;
                }
                upgrade_log_->append(QString("[离线演示] %1").arg(plan_->steps[demo_step_]));
                ++demo_step_;
                progress_->setValue(demo_step_ * 100 / plan_->steps.size());
                if (demo_step_ == plan_->steps.size())
                {
                    cancel_demo();
                    upgrade_status_->setText("演示结束 · 没有设备烧录、复位或升级结果");
                }
            });
    select_mode(UpgradeMode::App);
    auto *scroll = new QScrollArea;
    scroll->setObjectName("upgradeScroll");
    scroll->setWidgetResizable(true);
    scroll->setWidget(page);
    return scroll;
}

void MaintenanceWindow::select_mode(UpgradeMode mode)
{
    mode_ = mode;
    loader_path_->setEnabled(mode != UpgradeMode::App);
    app_path_->setEnabled(mode != UpgradeMode::Loader);
    loader_path_->parentWidget()
        ->findChild<QPushButton *>("chooseLoader")
        ->setEnabled(mode != UpgradeMode::App);
    app_path_->parentWidget()
        ->findChild<QPushButton *>("chooseApp")
        ->setEnabled(mode != UpgradeMode::Loader);
    route_->setText(mode == UpgradeMode::App ? "APP 通过 CAN FD 升级；Loader 与参数区保持保护。"
                    : mode == UpgradeMode::Loader
                        ? "Loader 使用 SWD 离线安装；保留当前 APP 和参数区。"
                        : "Loader 与 APP 配套离线安装；分区分别校验，禁止全片擦除，保留参数区。");
    invalidate_plan();
}

void MaintenanceWindow::invalidate_plan()
{
    if (demo_timer_)
    {
        cancel_demo();
    }
    plan_.reset();
    export_job_button_->setEnabled(false);
    demo_button_->setEnabled(false);
    validation_->setText("尚未校验 · 更改模式或文件后需要重新检查。");
}

void MaintenanceWindow::load_samples()
{
    synthetic_ = true;
    loader_path_->setText("内置合成 Loader · 不可烧录");
    app_path_->setText("内置合成 APP · 不可烧录");
    inspect_files();
}

void MaintenanceWindow::inspect_files()
{
    cancel_demo();
    plan_ = Upgrade_Inspect(mode_, loader_path_->text(), app_path_->text(), synthetic_);
    export_job_button_->setEnabled(plan_->valid);
    demo_button_->setEnabled(plan_->valid && connection_->currentIndex() == 0);
    progress_->setValue(0);
    upgrade_log_->clear();
    if (!plan_->valid)
    {
        validation_->setText("校验失败：" + plan_->error);
        upgrade_status_->setText("任务未准备，禁止继续");
        return;
    }
    validation_->setText((synthetic_ ? "演示样本检查通过 · 不可用于设备安装"
                                     : "本地文件检查通过 · 设备兼容性待核对") +
                         QString("\n%1；%2 个安装区域，参数区不在写域内。")
                             .arg(plan_->transport)
                             .arg(plan_->regions.size()));
    for (const auto &region : plan_->regions)
    {
        upgrade_log_->append(QString("%1    0x%2    %3 B")
                                 .arg(region.component)
                                 .arg(region.address, 8, 16, QChar('0'))
                                 .arg(region.bytes.size()));
    }
    for (const auto &check : plan_->checks)
    {
        upgrade_log_->append(check);
    }
    upgrade_status_->setText("任务已准备 · 可以导出，或运行离线演示");
}

void MaintenanceWindow::export_job()
{
    if (!plan_ || !plan_->valid)
    {
        return;
    }
    const auto parent = QFileDialog::getExistingDirectory(this, "选择 outputs 下的任务输出父目录");
    if (parent.isEmpty())
    {
        return;
    }
    try
    {
        const auto directory = QDir(parent).filePath(
            "upgrade-job-" + QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz"));
        const auto path = Upgrade_Export(*plan_, directory);
        upgrade_status_->setText("任务已导出，尚未执行设备安装");
        upgrade_log_->append(path);
    }
    catch (const std::exception &error)
    {
        upgrade_status_->setText(QString::fromUtf8(error.what()));
    }
}

void MaintenanceWindow::start_demo()
{
    if (!plan_ || !plan_->valid || connection_->currentIndex() != 0)
    {
        return;
    }
    demo_step_ = 0;
    progress_->setValue(0);
    upgrade_status_->setText("正在演示步骤 · 没有设备操作");
    demo_button_->setEnabled(false);
    export_job_button_->setEnabled(false);
    cancel_button_->setEnabled(true);
    demo_timer_->start();
}

void MaintenanceWindow::cancel_demo()
{
    if (demo_timer_ && demo_timer_->isActive())
    {
        demo_timer_->stop();
        upgrade_status_->setText("演示已取消 · 没有设备操作");
    }
    if (cancel_button_)
    {
        cancel_button_->setEnabled(false);
        demo_button_->setEnabled(plan_ && plan_->valid && connection_->currentIndex() == 0);
        export_job_button_->setEnabled(plan_ && plan_->valid);
    }
}

QWidget *MaintenanceWindow::build_parameter_page()
{
    auto *page = new QWidget;
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);
    auto *frame = card(layout, "活动参数 · 离线示例");
    auto *content = static_cast<QVBoxLayout *>(frame->layout());
    auto *actions = new QHBoxLayout;
    read_button_ = button("读取参数", "readParameters", "primary");
    write_button_ = button("写入选中参数", "writeParameter");
    save_button_ = button("保存到 Flash（演示）", "saveParameters");
    auto *export_snapshot = button("导出活动快照", "exportParameters");
    write_button_->setEnabled(false);
    save_button_->setEnabled(false);
    for (auto *item : {read_button_, write_button_, save_button_, export_snapshot})
    {
        actions->addWidget(item);
    }
    actions->addStretch();
    content->addLayout(actions);
    revision_label_ = label("活动 revision：未知    |    已保存 revision：未知", "subtitle");
    revision_label_->setObjectName("parameterRevision");
    content->addWidget(revision_label_);
    parameter_table_ = new QTableWidget;
    parameter_table_->setObjectName("parameterTable");
    const auto definitions = sandbox_.Sandbox_Definitions();
    parameter_table_->setColumnCount(6);
    parameter_table_->setRowCount(definitions.size());
    parameter_table_->setHorizontalHeaderLabels({"ID", "参数", "活动值", "单位", "新值", "权限"});
    parameter_table_->verticalHeader()->hide();
    parameter_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    parameter_table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    parameter_table_->setAlternatingRowColors(true);
    parameter_table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    parameter_table_->setSelectionMode(QAbstractItemView::SingleSelection);
    parameter_table_->setMinimumHeight(330);
    for (int row = 0; row < definitions.size(); ++row)
    {
        const auto &definition = definitions[row];
        const QStringList values{QString("0x%1").arg(definition.id, 2, 16, QChar('0')),
                                 definition.name,
                                 "未读取",
                                 definition.unit,
                                 "",
                                 "未知"};
        for (int column = 0; column < values.size(); ++column)
        {
            auto *item = new QTableWidgetItem(values[column]);
            item->setFlags(item->flags() & ~Qt::ItemIsEditable);
            parameter_table_->setItem(row, column, item);
        }
        parameter_table_->setRowHeight(row, 34);
    }
    content->addWidget(parameter_table_, 1);
    parameter_status_ = label("请先读取示例参数，读取后才能按权限编辑。", "subtitle");
    parameter_status_->setObjectName("parameterStatus");
    parameter_status_->setWordWrap(true);
    content->addWidget(parameter_status_);
    content->addWidget(label("写入只改变活动值；单独保存完成后才更新已保存 "
                             "revision。\n活动快照不是完整 Flash 参数区备份。",
                             "subtitle"));
    connect(read_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                read_parameters();
            });
    connect(write_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                write_parameter();
            });
    connect(save_button_,
            &QPushButton::clicked,
            this,
            [this]
            {
                save_parameters();
            });
    connect(export_snapshot,
            &QPushButton::clicked,
            this,
            [this]
            {
                export_parameters();
            });
    return page;
}

void MaintenanceWindow::set_parameter_status(const QString &text)
{
    parameter_status_->setText(text);
}

void MaintenanceWindow::read_parameters()
{
    if (connection_->currentIndex() != 0)
    {
        return;
    }
    parameter_snapshots_.clear();
    const auto definitions = sandbox_.Sandbox_Definitions();
    for (int row = 0; row < definitions.size(); ++row)
    {
        const auto reply = sandbox_.Sandbox_Read(definitions[row].id);
        if (!reply.value)
        {
            parameter_table_->item(row, 2)->setText("不支持");
            continue;
        }
        const auto value = *reply.value;
        parameter_snapshots_[row] = value;
        const auto text = QString::number(scalar(value) / definitions[row].scale, 'g', 10);
        parameter_table_->item(row, 2)->setText(text);
        parameter_table_->item(row, 4)->setText(text);
        auto flags = parameter_table_->item(row, 4)->flags();
        if (value.access & 2)
        {
            flags |= Qt::ItemIsEditable;
        }
        else
        {
            flags &= ~Qt::ItemIsEditable;
        }
        parameter_table_->item(row, 4)->setFlags(flags);
        parameter_table_->item(row, 5)->setText(value.access & 2 ? "停机可写（示例）" : "只读");
    }
    const auto revision = sandbox_.Sandbox_Read(0).value->raw;
    revision_label_->setText(
        QString("活动 revision：%1    |    已保存 revision：%2    |    来源：离线示例")
            .arg(revision)
            .arg(sandbox_.Sandbox_PersistedRevision()));
    const bool pending = sandbox_.Sandbox_SavePending();
    write_button_->setEnabled(!pending);
    save_button_->setEnabled(!pending);
    set_parameter_status(pending ? "示例参数读取完成；保存作业仍在等待完成。"
                                 : "示例参数读取完成；没有设备数据。");
}

void MaintenanceWindow::write_parameter()
{
    const int row = parameter_table_->currentRow();
    if (connection_->currentIndex() != 0 || !parameter_snapshots_.contains(row))
    {
        set_parameter_status("请先读取并选中一个参数。");
        return;
    }
    auto value = parameter_snapshots_.value(row);
    if (!(value.access & 2))
    {
        set_parameter_status("该参数只读，不能通过通用写入修改。");
        return;
    }
    const auto definitions = sandbox_.Sandbox_Definitions();
    bool valid = false;
    const auto number = parameter_table_->item(row, 4)->text().toDouble(&valid);
    const double raw = number * definitions[row].scale;
    const double minimum = value.value_type == 1 ? std::numeric_limits<qint32>::min() : 0;
    const double maximum = value.value_type == 1 ? std::numeric_limits<qint32>::max()
                                                 : std::numeric_limits<quint32>::max();
    if (!valid || !std::isfinite(raw) || raw < minimum || raw > maximum ||
        std::abs(raw - std::round(raw)) > 0.00001)
    {
        set_parameter_status("数值无效、超出线路范围或精度不符合当前缩放。");
        return;
    }
    value.raw = quint32(qint64(std::round(raw)));
    const auto reply = sandbox_.Sandbox_Write(value);
    if (reply.result != 0 || !reply.value)
    {
        set_parameter_status(reason(reply));
        return;
    }
    const auto readback = sandbox_.Sandbox_Read(value.id);
    if (!readback.value || readback.value->raw != reply.value->raw ||
        readback.value->revision != reply.value->revision)
    {
        set_parameter_status("读回不一致，写入结果未知。");
        return;
    }
    read_parameters();
    set_parameter_status("活动值已写入并读回（演示）；尚未保存到 Flash。");
}

void MaintenanceWindow::save_parameters()
{
    if (connection_->currentIndex() != 0 || parameter_snapshots_.isEmpty())
    {
        return;
    }
    const auto expected = parameter_snapshots_.first().revision;
    const auto reply = sandbox_.Sandbox_Save(expected);
    if (reply.result != 1)
    {
        set_parameter_status(reason(reply));
        return;
    }
    save_button_->setEnabled(false);
    write_button_->setEnabled(false);
    set_parameter_status("保存已受理（演示），尚未完成持久化。");
    QTimer::singleShot(450,
                       this,
                       [this]
                       {
                           const auto completed = sandbox_.Sandbox_CompleteSave(true);
                           if (connection_->currentIndex() == 0)
                           {
                               read_parameters();
                               set_parameter_status(
                                   completed.result == 0
                                       ? "保存作业已完成（演示）；没有设备 Flash 操作。"
                                       : reason(completed));
                           }
                       });
}

void MaintenanceWindow::export_parameters()
{
    if (parameter_snapshots_.isEmpty())
    {
        set_parameter_status("先读取活动值，再导出快照。");
        return;
    }
    const auto path =
        QFileDialog::getSaveFileName(this, "导出活动参数快照到 outputs", {}, "JSON (*.json)");
    if (path.isEmpty())
    {
        return;
    }
    QJsonArray values;
    for (const auto &value : parameter_snapshots_)
    {
        values.append(QJsonObject{{"id", value.id},
                                  {"type", value.value_type},
                                  {"raw_u32", qint64(value.raw)},
                                  {"revision", qint64(value.revision)},
                                  {"access", value.access}});
    }
    QSaveFile file(path);
    const auto bytes = QJsonDocument(QJsonObject{{"source", "offline_sandbox"},
                                                 {"device_read", false},
                                                 {"raw_flash_backup", false},
                                                 {"values", values}})
                           .toJson();
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit())
    {
        set_parameter_status("快照导出失败，请检查输出目录。");
        return;
    }
    set_parameter_status("已导出示例活动值快照；它不替代完整参数区备份。");
}
} // namespace yg::host
