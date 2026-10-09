/** 基础工作区回归：控制门禁、停止与任务终态、完整记录以及共享参数草稿。 */
#include "gui/maintenance_window.h"
#include "services/capture_csv.h"
#include "services/device_readback.h"
extern "C"
{
#include "shared/protocol/yg_protocol_byte_order.h"
#include "shared/protocol/yg_protocol_can_id.h"
#include "shared/protocol/yg_protocol_frame_codec.h"
}
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFile>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QStackedWidget>
#include <QStandardItemModel>
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
QByteArray reply(uint8_t node, uint16_t type, uint16_t sequence, const QByteArray &payload)
{
    yg_protocol_frame_view_t message{};
    message.version = YG_PROTOCOL_VERSION;
    message.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    message.source_node = node;
    message.destination_node = 2;
    message.message_type = type;
    message.sequence = sequence;
    message.payload_length = uint32_t(payload.size());
    message.payload = reinterpret_cast<const uint8_t *>(payload.constData());
    QByteArray frame(64, 0);
    size_t written = 0;
    check(yg_protocol_frame_encode(
              &message, reinterpret_cast<uint8_t *>(frame.data()), 64, &written) == YG_PROTOCOL_OK,
          "test reply encode failed");
    for (const int dlc : {20, 24, 32, 48, 64})
    {
        if (written <= size_t(dlc))
        {
            frame.resize(dlc);
            return frame;
        }
    }
    throw std::runtime_error("test reply too large");
}
void device_readback_checks()
{
    DeviceReadback device;
    check(device.Device_Reset(3), "node reset rejected");
    yg_protocol_can_id_fields_t fields{3, 3, 2};
    uint32_t can_id = 0;
    check(yg_protocol_can_id_encode(&fields, &can_id) == YG_PROTOCOL_OK, "CAN ID encode");
    QByteArray info(32, 0);
    info[0] = 0x57;
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(info.data() + 12), 0x59473236);
    YgProtocol_WriteU16Le(reinterpret_cast<uint8_t *>(info.data() + 16), 1);
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(info.data() + 20), 0x00010100);
    check(device.Device_Begin(1, 10, 0, 100), "info query registration failed");
    auto info_frame = reply(3, 1, 10, info);
    check(!device.Device_Accept(can_id, reply(3, 1, 10, QByteArray(32, 0)), 105),
          "zero UID accepted as physical identity");
    auto bad = info_frame;
    bad[bad.size() - 1] = 1;
    auto bad_crc = info_frame;
    bad_crc[16] ^= 1;
    yg_protocol_can_id_fields_t wrong_node{3, 4, 2};
    uint32_t wrong_id = 0;
    check(yg_protocol_can_id_encode(&wrong_node, &wrong_id) == YG_PROTOCOL_OK,
          "wrong CAN ID encode");
    check(!device.Device_Accept(can_id, bad, 110) && !device.Device_Accept(can_id, bad_crc, 110) &&
              !device.Device_Accept(wrong_id, info_frame, 110) &&
              !device.Device_Accept(can_id, reply(3, 1, 11, info), 110),
          "bad padding, CRC, node or sequence accepted");
    check(device.Device_Accept(can_id, info_frame, 110), "valid info rejected");
    QByteArray identity(32, 0);
    identity.replace(0, 7, "ABC-123");
    identity[20] = 1;
    identity[21] = 3;
    identity[22] = 1;
    identity[23] = 1;
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(identity.data() + 16), 7);
    auto malformed_sn = identity;
    malformed_sn[0] = '!';
    check(device.Device_Begin(1, 11, 1, 120) &&
              !device.Device_Accept(can_id, reply(3, 1, 11, malformed_sn), 125) &&
              device.Device_Accept(can_id, reply(3, 1, 11, identity), 130),
          "role and motor type rejected");
    QByteArray caps(16, 0);
    YgProtocol_WriteU16Le(reinterpret_cast<uint8_t *>(caps.data() + 6), 64);
    check(device.Device_Begin(2, 12, 0, 140) &&
              device.Device_Accept(can_id, reply(3, 2, 12, caps), 150),
          "capabilities rejected");
    const auto identified = device.Device_View(150);
    check(identified.identity_fresh && identified.uid.startsWith("57") &&
              identified.sn == "ABC-123" && identified.motor_type == 7 &&
              identified.supported_modes == 0,
          "identity or disabled capabilities fabricated");
    QByteArray device_status(16, 0);
    YgProtocol_WriteU16Le(reinterpret_cast<uint8_t *>(device_status.data()), 6);
    device_status[4] = 1;
    check(device.Device_Begin(3, 18, 0, 150) &&
              device.Device_Accept(can_id, reply(3, 3, 18, device_status), 151) &&
              !device.Device_View(151).boot_epoch,
          "unavailable epoch fabricated a startup identity");
    QByteArray geometry(32, 0);
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(geometry.data() + 8), 0x000C10D0);
    YgProtocol_WriteU32Le(reinterpret_cast<uint8_t *>(geometry.data() + 12), 12);
    check(device.Device_Begin(1, 16, 2, 151) &&
              device.Device_Accept(can_id, reply(3, 1, 16, geometry), 152) &&
              device.Device_View(152).parameter_schema == 12,
          "parameter schema was inferred instead of read back");
    QByteArray feedback =
        QByteArray::fromHex("0000000004030201E803000030F8FFFFB80B000060F0FFFFFF0D6009DC0550FB"
                            "4C04E6090080881306FF030603");
    check(feedback.size() == 45 && device.Device_Begin(108, 13, 0, 160), "feedback query invalid");
    auto invalid_feedback = feedback;
    invalid_feedback[24] &= ~uint8_t(1U << 7U);
    check(!device.Device_Accept(can_id, reply(3, 108, 13, invalid_feedback), 165),
          "invalid measurement bit accepted");
    check(device.Device_Accept(can_id, reply(3, 108, 13, feedback), 170),
          "golden feedback rejected");
    const auto view = device.Device_View(170);
    check(view.feedback_fresh && view.motor_state == 6 && view.position_rad == -2 &&
              view.speed_rad_s == -4 && view.iq_a == 1.1 && !device.Device_View(671).feedback_fresh,
          "SI feedback or expiry wrong");
    check(
        device.Device_Begin(110, 14, 0, 680) &&
            device.Device_Accept(can_id, reply(3, 110, 14, QByteArray::fromHex("01000000")), 690) &&
            !device.Device_View(690).stop_confirmed_recently,
        "STOP accepted was treated as output disabled");
    check(device.Device_Begin(110, 15, 0, 700) &&
              !device.Device_Accept(
                  can_id, reply(3, 110, 15, QByteArray::fromHex("00000100")), 705) &&
              device.Device_Accept(can_id, reply(3, 110, 15, QByteArray(4, 0)), 710) &&
              device.Device_View(710).stop_confirmed_recently &&
              !device.Device_View(1211).stop_confirmed_recently,
          "STOP detail or confirmation expiry wrong");
    check(device.Device_Begin(108, 17, 0, 720) &&
              device.Device_Accept(can_id, reply(3, 108, 17, feedback), 730) &&
              !device.Device_View(730).stop_confirmed_recently,
          "later RUNNING feedback preserved obsolete STOP confirmation");
    check(!device.Device_View(5151).identity_fresh && device.Device_View(5151).uid.isEmpty(),
          "expired identity still advertised");

    DeviceReadback changed;
    check(changed.Device_Reset(3) && changed.Device_Begin(1, 21, 0, 100) &&
              changed.Device_Accept(can_id, reply(3, 1, 21, info), 101) &&
              changed.Device_Begin(1, 22, 1, 102) &&
              changed.Device_Accept(can_id, reply(3, 1, 22, identity), 103) &&
              changed.Device_Begin(2, 23, 0, 104) &&
              changed.Device_Accept(can_id, reply(3, 2, 23, caps), 105) &&
              changed.Device_Begin(1, 24, 2, 106) &&
              changed.Device_Accept(can_id, reply(3, 1, 24, geometry), 107),
          "session setup failed");
    auto another_uid = info;
    another_uid[0] = 0x58;
    check(changed.Device_Begin(1, 25, 0, 108) &&
              changed.Device_Accept(can_id, reply(3, 1, 25, another_uid), 109),
          "new UID rejected");
    const auto after_uid = changed.Device_View(109);
    check(after_uid.uid.startsWith("58") && after_uid.sn.isEmpty() && !after_uid.parameter_schema &&
              !after_uid.features && !after_uid.identity_fresh,
          "new UID retained old identity or capabilities");
    check(changed.Device_Begin(1, 26, 1, 110) &&
              changed.Device_Accept(can_id, reply(3, 1, 26, identity), 111) &&
              changed.Device_Begin(2, 27, 0, 112) &&
              changed.Device_Accept(can_id, reply(3, 2, 27, caps), 113),
          "new UID discovery failed");
    QByteArray epoch_status(16, 0);
    epoch_status[4] = 1;
    epoch_status[5] = 1;
    YgProtocol_WriteU16Le(reinterpret_cast<uint8_t *>(epoch_status.data() + 6), 1);
    YgProtocol_WriteU64Le(reinterpret_cast<uint8_t *>(epoch_status.data() + 8), 9);
    check(changed.Device_Begin(3, 28, 0, 114) &&
              changed.Device_Accept(can_id, reply(3, 3, 28, epoch_status), 115) &&
              !changed.Device_View(115).identity_fresh,
          "first valid epoch retained uncorrelated discovery");
    check(changed.Device_Begin(1, 29, 0, 116) &&
              changed.Device_Accept(can_id, reply(3, 1, 29, another_uid), 117) &&
              changed.Device_Begin(1, 30, 1, 118) &&
              changed.Device_Accept(can_id, reply(3, 1, 30, identity), 119) &&
              changed.Device_Begin(2, 31, 0, 120) &&
              changed.Device_Accept(can_id, reply(3, 2, 31, caps), 121) &&
              changed.Device_Begin(3, 32, 0, 122) &&
              changed.Device_Accept(can_id, reply(3, 3, 32, epoch_status), 123) &&
              changed.Device_View(123).identity_fresh,
          "unchanged epoch lost discovery");
    YgProtocol_WriteU64Le(reinterpret_cast<uint8_t *>(epoch_status.data() + 8), 10);
    check(changed.Device_Begin(3, 33, 0, 124) &&
              changed.Device_Accept(can_id, reply(3, 3, 33, epoch_status), 125) &&
              !changed.Device_View(125).identity_fresh && changed.Device_View(125).uid.isEmpty() &&
              changed.Device_View(125).boot_epoch == 10,
          "changed epoch retained old-session readback");
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
    check(!qobject_cast<QStandardItemModel *>(
               window.findChild<QComboBox *>("identificationKind")->model())
               ->item(int(TaskKind::Inductance) - 4)
               ->isEnabled(),
          "missing device inductance capability left enabled");
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
    press("disconnectDevice");
    press("connectionSettings");
    window.findChild<QComboBox *>("connectionMode")->setCurrentIndex(1);
    check(window.findChild<QProcess *>() == nullptr,
          "selecting CAN opened a device helper without explicit connect");
    check(!window.findChild<QLineEdit *>("quickDraft")->isEnabled(),
          "CAN read-only page retained an editable parameter draft");
    press("connectDevice");
    check(window.findChild<QLabel *>("footer")->text().contains("缺少 Python") &&
              window.findChild<QProcess *>() == nullptr,
          "missing SDK configuration opened a device helper");
    press("homeNav");
    QByteArray feedback =
        QByteArray::fromHex("0000000004030201E803000030F8FFFFB80B000060F0FFFFFF0D6009DC0550FB"
                            "4C04E6090080881306FF030603");
    yg_protocol_can_id_fields_t fields{3, 3, 2};
    uint32_t can_id = 0;
    check(yg_protocol_can_id_encode(&fields, &can_id) == YG_PROTOCOL_OK,
          "GUI readback CAN ID encode");
    check(window.Window_BeginDeviceRead(108, 35, 0) &&
              window.Window_ObserveDeviceFrame(can_id, reply(3, 108, 35, feedback)),
          "GUI readback seam rejected wire reply");
    check(window.findChild<QLabel *>("homeTelemetry")->text().contains("八态 6") &&
              !window.findChild<QPushButton *>("enableMotor")->isEnabled() &&
              !window.findChild<QPushButton *>("stopMotor")->isEnabled(),
          "wire replay was shown as demo or gained power controls");
}
} // namespace
int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    try
    {
        device_readback_checks();
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
