/** 主机直接相关离线检查：真实共享包校验、参数黄金字节、作业结果及页面门禁。 */
#include "gui/maintenance_window.h"
#include "protocol/parameter_wire.h"
#include <QApplication>
#include <QComboBox>
#include <QCheckBox>
#include <QStackedWidget>
#include <QDir>
#include <QEventLoop>
#include <QFile>
#include <QJsonDocument>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QTemporaryDir>
#include <QTimer>
#include <iostream>
#include <stdexcept>

using namespace yg::host;
namespace
{
void check(bool value, const char *message)
{
    if (!value)
    {
        throw std::runtime_error(message);
    }
}

template <typename Action> void rejects(Action action)
{
    bool rejected = false;
    try
    {
        action();
    }
    catch (const std::exception &)
    {
        rejected = true;
    }
    check(rejected, "invalid input was accepted");
}

void file_write(const QString &path, const QByteArray &bytes)
{
    QFile file(path);
    check(file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size(),
          "fixture write failed");
}

void service_checks()
{
    for (auto mode : {UpgradeMode::Loader, UpgradeMode::App, UpgradeMode::Combined})
    {
        auto plan = Upgrade_Inspect(mode, {}, {}, true);
        check(plan.valid && plan.synthetic,
              "synthetic sample must pass actual shared image validator");
        check(plan.regions.size() == (mode == UpgradeMode::Combined ? 2 : 1),
              "incorrect write regions");
        check(mode == UpgradeMode::App ? plan.regions[0].component == "APP"
                                       : plan.regions[0].component == "Loader",
              "mode selected wrong component");
        QTemporaryDir directory;
        const auto output = directory.filePath("job");
        const auto exported = Upgrade_Export(plan, output);
        QFile job(exported);
        check(job.open(QIODevice::ReadOnly), "job export absent");
        const auto report = QJsonDocument::fromJson(job.readAll()).object();
        check(report["status"] == "prepared_only" && !report["ready_to_execute"].toBool() &&
                  !report["hardware_accessed"].toBool(),
              "export claimed device execution");
        rejects(
            [&]
            {
                Upgrade_Export(plan, output);
            });
        if (mode != UpgradeMode::Loader)
        {
            const auto app_path = QDir(output).filePath("image.bin");
            check(Upgrade_Inspect(UpgradeMode::App, {}, app_path).valid,
                  "exported APP revalidation failed");
            auto damaged = plan.regions.last().bytes;
            damaged[damaged.size() - 1] ^= 1;
            file_write(app_path, damaged);
            check(!Upgrade_Inspect(UpgradeMode::App, {}, app_path).valid, "bad image CRC accepted");
        }
    }
    check(!Upgrade_Inspect(UpgradeMode::Combined, "missing.bin", "missing.bin").valid,
          "missing files accepted");
    const auto read = Parameter_ReadPayload(1);
    check(read.toHex() == "01000000", "READ golden mismatch");
    check(Parameter_WritePayload({1, 1, 3, 5, 1250}).toHex() == "0101040005000000e2040000",
          "WRITE golden mismatch");
    auto reply =
        Parameter_DecodeReply(120, QByteArray::fromHex("00000000000204010100008001000080"), 0);
    check(reply.value && reply.value->raw == 0x80000001 && reply.value->revision == 0x80000001,
          "u32 high bit lost");
    reply = Parameter_DecodeReply(121, QByteArray::fromHex("01000000"), 1);
    check(reply.result == 1 && !reply.value, "ACCEPTED invented active value");
    rejects(
        []
        {
            Parameter_DecodeReply(120, QByteArray::fromHex("01000000"), 1);
        });
    rejects(
        []
        {
            Parameter_DecodeReply(120, QByteArray::fromHex("000000000101040805000000e2040000"), 1);
        });
    rejects(
        []
        {
            Parameter_DecodeReply(
                121, QByteArray::fromHex("000000000102040006000000e2040000"), 1, 1);
        });
    ParameterSandbox sandbox;
    auto value = *sandbox.Sandbox_Read(1).value;
    value.raw = 12000;
    const auto written = sandbox.Sandbox_Write(value);
    check(written.result == 0 && written.value->revision == 6 &&
              sandbox.Sandbox_PersistedRevision() == 5,
          "WRITE claimed persistence");
    check(sandbox.Sandbox_Write(value).detail == 3, "old revision accepted");
    check(sandbox.Sandbox_Write(*sandbox.Sandbox_Read(3).value).detail == 2,
          "readonly parameter written");
    const auto pending = sandbox.Sandbox_Save(6);
    check(pending.result == 1 && pending.persisted_revision == 5,
          "SAVE accepted claimed completed");
    check(sandbox.Sandbox_Write(*sandbox.Sandbox_Read(1).value).result == 6,
          "save did not own mutation");
    check(sandbox.Sandbox_CompleteSave(false).result == 8 &&
              sandbox.Sandbox_PersistedRevision() == 5,
          "failed save changed persistence");
    check(sandbox.Sandbox_Save(6).result == 1 &&
              sandbox.Sandbox_CompleteSave(true).persisted_revision == 6,
          "save completion not observable");
}

void gui_checks()
{
    MaintenanceWindow window;
    window.show();
    QApplication::processEvents();
    auto find_button = [&](const char *name)
    {
        return window.findChild<QPushButton *>(name);
    };
    check(!find_button("exportJob")->isEnabled(), "unchecked files enabled export");
    for (int mode = 0; mode < 3; ++mode)
    {
        find_button(qPrintable(QString("mode%1").arg(mode)))->click();
        find_button("loadSamples")->click();
        check(find_button("exportJob")->isEnabled(), "valid mode did not enable job export");
        check(window.findChild<QLineEdit *>("loaderPath")->isEnabled() == (mode != 1),
              "loader path gate");
        check(window.findChild<QLineEdit *>("appPath")->isEnabled() == (mode != 0),
              "APP path gate");
    }
    find_button("startDemo")->click();
    check(find_button("cancelDemo")->isEnabled(), "demo not started");
    find_button("cancelDemo")->click();
    check(window.findChild<QLabel *>("upgradeStatus")->text().contains("取消"),
          "demo cancel not explicit");
    find_button("parameterNav")->click();
    find_button("connectDevice")->click();
    window.findChild<QCheckBox *>("unlockWrites")->setChecked(true);
    find_button("parameterNav")->click();
    find_button("readParameters")->click();
    auto *table = window.findChild<QTableWidget *>("parameterTable");
    check(table->item(1, 2)->text() == "10", "parameter source/display mismatch");
    table->selectRow(1);
    table->item(1, 4)->setText("12");
    find_button("writeParameter")->click();
    check(table->item(1, 2)->text() == "12", "active value not read back");
    check(window.findChild<QLabel *>("parameterStatus")->text().contains("尚未保存"),
          "UI claimed WRITE persistence");
    find_button("saveParameters")->click();
    check(window.findChild<QLabel *>("parameterStatus")->text().contains("尚未完成"),
          "UI claimed SAVE acceptance completion");
    find_button("readParameters")->click();
    check(!find_button("writeParameter")->isEnabled() &&
              !find_button("saveParameters")->isEnabled(),
          "READ enabled mutation during SAVE");
    QEventLoop wait;
    QTimer::singleShot(600, &wait, &QEventLoop::quit);
    wait.exec();
    check(window.findChild<QLabel *>("parameterStatus")->text().contains("作业已完成"),
          "save terminal UI absent");
    window.findChild<QComboBox *>("connectionMode")->setCurrentIndex(1);
    check(!find_button("writeParameter")->isEnabled() && !find_button("startDemo")->isEnabled(),
          "offline model exposed as physical operation");
}
} // namespace

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    try
    {
        service_checks();
        gui_checks();
        std::cout << "host upgrade/parameter services and GUI offline PASS; no hardware access\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
