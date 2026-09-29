/** 基础工作区回归：控制门禁、停止与任务终态、完整记录以及共享参数草稿。 */
#include "gui/maintenance_window.h"
#include "services/capture_csv.h"
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QTableWidget>
#include <QTemporaryDir>
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
void service_checks()
{
    WorkbenchSession session;
    check(!session.Session_Enable(true) && !session.Session_Stream(true),
          "disconnected actions accepted");
    check(session.Session_Connect(3, DemoScenario::Normal), "connect failed");
    check(!session.Session_View().enabled && !session.Session_View().streaming,
          "connect performed implicit actions");
    check(!session.Session_Enable(true), "locked model enabled");
    session.Session_Unlock(true);
    check(session.Session_Enable(true), "unlock did not allow explicit enable");
    check(!session.Session_Move(MotionMode::Current, 4), "out of range current accepted");
    check(session.Session_Move(MotionMode::Position, 1), "position command rejected");
    check(session.Session_Stream(true) && session.Session_Record(true), "capture rejected");
    session.Session_Tick(0.2);
    check(session.Session_Recording().size() == 10, "model capture is not 50 Hz");
    check(session.Session_Recording().back().position_rad > 0, "motion has no model response");
    QTemporaryDir directory;
    const auto path = directory.filePath("capture.csv");
    Capture_Write(path, session.Session_Recording());
    const auto replay = Capture_Read(path);
    check(replay.size() == 10 && replay.back().demo_node == 3 &&
              replay.back().position_rad == session.Session_Recording().back().position_rad &&
              replay.back().iq_reference_a == session.Session_Recording().back().iq_reference_a,
          "CSV SI roundtrip changed samples");
    QFile bad(directory.filePath("bad.csv"));
    check(bad.open(QIODevice::WriteOnly), "bad fixture open");
    bad.write("time,position\n0,1\n");
    bad.close();
    bool rejected = false;
    try
    {
        Capture_Read(bad.fileName());
    }
    catch (const std::exception &)
    {
        rejected = true;
    }
    check(rejected, "unknown CSV source accepted");
    session.Session_Stop();
    check(session.Session_MotionParameters({1, 0.5, 2}), "stopped model parameters rejected");
    session.Session_Enable(true);
    check(!session.Session_Move(MotionMode::Current, 0.6) &&
              !session.Session_MotionParameters({2, 1, 3}),
          "confirmed current limit or stopped-only model mutation bypassed");
    session.Session_Stop();
    check(!session.Session_View().moving && !session.Session_View().enabled &&
              session.Session_View().target == 0,
          "stop retained old motion target");
    check(session.Session_Enable(true) && !session.Session_View().moving,
          "reenable resumed old motion");
    session.Session_Stop();
    check(session.Session_StartTask(TaskKind::Sensor), "task rejected");
    check(!session.Session_StartTask(TaskKind::Resistance) && !session.Session_Enable(true),
          "busy task lost ownership");
    const auto run_id = session.Session_View().task.run_id;
    session.Session_CancelTask();
    for (int i = 0; i < 30; ++i)
    {
        session.Session_Tick(0.2);
    }
    check(session.Session_View().task.phase == TaskPhase::Cancelled &&
              session.Session_View().task.result.isEmpty(),
          "cancelled task became completed");
    check(session.Session_StartTask(TaskKind::Inductance), "second task rejected");
    for (int i = 0; i < 21; ++i)
    {
        session.Session_Tick(0.2);
    }
    check(session.Session_View().task.run_id > run_id &&
              session.Session_View().task.phase == TaskPhase::Completed &&
              !session.Session_View().task.applied_to_demo,
          "completion invented application or reused run");
    check(session.Session_ApplyTask(), "candidate application rejected");
    session.Session_Disconnect();
    check(!session.Session_Recording().empty() && !session.Session_View().recording,
          "disconnect discarded record");
    for (const auto scenario : {DemoScenario::ReadOnly, DemoScenario::Loader, DemoScenario::Fault})
    {
        WorkbenchSession limited;
        limited.Session_Connect(4, scenario);
        limited.Session_Unlock(true);
        check(!limited.Session_Enable(true) && !limited.Session_StartTask(TaskKind::Sensor),
              "scenario gate bypassed");
        if (scenario == DemoScenario::Fault)
        {
            check(limited.Session_ClearFault() && !limited.Session_View().enabled,
                  "fault clear enabled model");
        }
    }
    ParameterSandbox readonly(4, false);
    check(readonly.Sandbox_Read(3).value->raw == 4 && readonly.Sandbox_Read(1).value->access == 1,
          "readonly scenario exposes write access or wrong node");
    check(readonly.Sandbox_Write(*readonly.Sandbox_Read(1).value).detail == 2,
          "readonly model accepted write");
    WorkbenchSession bounded;
    bounded.Session_Connect(3, DemoScenario::Normal);
    bounded.Session_Stream(true);
    bounded.Session_Record(true);
    for (int i = 0; i < 3002; ++i)
    {
        bounded.Session_Tick(0.2);
    }
    check(bounded.Session_Recording().size() == 30000 && !bounded.Session_View().recording &&
              bounded.Session_Samples().size() == 3000,
          "capture/display capacity not bounded");
}
void gui_checks()
{
    MaintenanceWindow window;
    window.resize(1366, 768);
    window.show();
    QApplication::processEvents();
    const auto press = [&](const char *name)
    {
        auto *button = window.findChild<QPushButton *>(name);
        check(button != nullptr, "missing original navigation/action");
        button->click();
    };
    auto *pages = window.findChild<QStackedWidget *>("workspacePages");
    check(pages->count() == 8 && pages->currentIndex() == 0,
          "connection plus seven workspaces not restored");
    press("connectDevice");
    check(pages->currentIndex() == 1 &&
              !window.findChild<QPushButton *>("executeMotion")->isEnabled(),
          "connection skipped home or enabled motion");
    press("workbenchNav");
    check(window.findChild<QWidget *>("scope")->isVisible() &&
              window.findChild<QPushButton *>("stopMotor")->isVisible(),
          "primary scope/stop invisible");
    window.findChild<QCheckBox *>("unlockWrites")->setChecked(true);
    press("streamStart");
    press("recordStart");
    press("enableMotor");
    press("executeMotion");
    for (int i = 0; i < 5; ++i)
    {
        window.Window_AdvanceDemo(0.2);
    }
    window.findChild<QCheckBox *>("freezeScope")->setChecked(true);
    press("diagnosticsNav");
    window.Window_AdvanceDemo(0.2);
    check(window.findChild<QLabel *>("recordStatus")->text().contains("正在记录"),
          "page switch/freeze ended capture");
    press("stopMotor");
    press("workbenchNav");
    press("quickRead");
    auto *table = window.findChild<QTableWidget *>("parameterTable");
    table->item(1, 4)->setText("12");
    table->item(2, 4)->setText("2.5");
    check(window.findChild<QLineEdit *>("quickDraft")->text() == "12",
          "quick/full drafts diverged");
    press("quickApply");
    check(table->item(1, 2)->text() == "12" && table->item(2, 4)->text() == "2.5",
          "apply lost another parameter draft");
    press("calibrationNav");
    press("startCalibration");
    window.Window_AdvanceDemo(0.2);
    press("identificationNav");
    check(!window.findChild<QPushButton *>("startIdentification")->isEnabled(),
          "task lost exclusivity across page switch");
    press("stopMotor");
    press("calibrationNav");
    check(window.findChild<QLabel *>("calibrationPhase")->text().contains("已取消"),
          "global stop did not cancel task");
    press("disconnectDevice");
    press("connectionSettings");
    window.findChild<QComboBox *>("demoScenario")->setCurrentIndex(1);
    press("connectDevice");
    window.findChild<QCheckBox *>("unlockWrites")->setChecked(true);
    press("parameterNav");
    press("readParameters");
    check(!window.findChild<QPushButton *>("writeParameter")->isEnabled() &&
              !(table->item(1, 4)->flags() & Qt::ItemIsEditable),
          "readonly scene allows parameter edit/write");
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
        std::cout << "workbench/session/record/shared-draft offline PASS; no hardware access\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
