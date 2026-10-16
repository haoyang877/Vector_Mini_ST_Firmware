/** 单页核心离线检查：假 CAN 辅助进程、序号和 STOP 最终确认。 */
#include "gui/maintenance_window.h"
#include "services/device_readback.h"
#include "services/upgrade_plan.h"
extern "C"
{
#include "shared/protocol/yg_protocol_byte_order.h"
#include "shared/protocol/yg_protocol_can_id.h"
#include "shared/protocol/yg_protocol_frame_codec.h"
}
#include <QApplication>
#include <QCheckBox>
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QSpinBox>
#include <QSettings>
#include <QTemporaryDir>
#include <QThread>
#include <functional>
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

int fake_reader()
{
    yg_protocol_can_id_fields_t address{3, 1, 2};
    uint32_t can_id = 0;
    check(yg_protocol_can_id_encode(&address, &can_id) == YG_PROTOCOL_OK,
          "fake reader CAN ID encode");
    int starts = 0;
    int stops = 0;
    const auto scenario = qEnvironmentVariable("YG_HOST_FAKE_SCENARIO");
    bool running = false;
    std::cout << "{\"event\":\"ready\",\"node\":1}\n" << std::flush;
    std::string line;
    while (std::getline(std::cin, line))
    {
        const auto request = QJsonDocument::fromJson(QByteArray::fromStdString(line)).object();
        const int sequence = request.value("sequence").toInt();
        QJsonObject result;
        if (request.contains("type"))
        {
            const int type = request.value("type").toInt();
            const int page = request.value("page").toInt();
            QByteArray payload(type == 108 ? 45 : type == 1 ? 32 : 16, 0);
            auto *data = reinterpret_cast<uint8_t *>(payload.data());
            if (type == 1 && page == 0)
            {
                payload.replace(
                    0,
                    12,
                    QByteArray::fromHex(scenario == "second-device" ||
                                                (scenario == "changed-uid" && starts > 0)
                                            ? "5800460010504e4831303320"
                                            : "5700460010504e4831303320"));
                data = reinterpret_cast<uint8_t *>(payload.data());
                YgProtocol_WriteU32Le(data + 12, scenario == "wrong-product" ? 99 : 0x59473236);
                YgProtocol_WriteU16Le(data + 16, scenario == "wrong-hardware" ? 99 : 1);
                YgProtocol_WriteU32Le(data + 20, 0x00010301);
                YgProtocol_WriteU32Le(data + 24, 0x00010000);
            }
            else if (type == 1 && page == 1)
            {
                payload.replace(0, 4, "FAKE");
                data = reinterpret_cast<uint8_t *>(payload.data());
                data[20] = 1;
                data[21] = 1;
                data[22] = 1;
                data[23] = 1;
            }
            else if (type == 1 && page == 2)
            {
                YgProtocol_WriteU32Le(data + 8, scenario == "wrong-abi" ? 99 : 0x000C10D0);
                YgProtocol_WriteU32Le(data + 12, scenario == "wrong-schema" ? 99 : 12);
            }
            else if (type == 2)
            {
                YgProtocol_WriteU16Le(data + 6, 64);
            }
            else if (type == 3)
            {
                YgProtocol_WriteU16Le(data, 6);
                data[4] = 1;
            }
            else if (type == 108)
            {
                YgProtocol_WriteU16Le(data + 24, 0x0FFF);
                YgProtocol_WriteU16Le(data + 26, 3300);
                data[42] = 1;
                data[43] = running ? 6 : 5;
                data[44] = running ? 2 : 0;
            }
            else
            {
                return 2;
            }
            result = {{"event", "frame"},
                      {"type", type},
                      {"sequence", sequence},
                      {"can_id", int(can_id)},
                      {"data", QString::fromLatin1(reply(1, type, sequence, payload).toHex())}};
        }
        else
        {
            const auto operation = request.value("op").toString();
            const int type = operation == "start_speed" ? 117 : operation == "stop" ? 110 : 116;
            int code = 1;
            int detail = 0;
            if (type == 117)
            {
                code = ++starts == 1 ? 1 : 2;
                running = code == 1;
            }
            else if (type == 110)
            {
                code = ++stops == 1 ? 1 : 0;
                detail = code == 1 ? 1 : 0;
                running = code == 1;
            }
            result = {{"event", "result"},
                      {"type", type},
                      {"sequence", sequence},
                      {"result", code},
                      {"detail", detail}};
        }
        std::cout << QJsonDocument(result).toJson(QJsonDocument::Compact).constData() << '\n'
                  << std::flush;
    }
    return 0;
}

bool wait_for(const std::function<bool()> &condition, int timeout_ms)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < timeout_ms)
    {
        QApplication::processEvents();
        if (condition())
        {
            return true;
        }
        QThread::msleep(10);
    }
    return false;
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
              view.speed_rad_s == -4 && view.iq_a == 1.1 && view.bus_current_a == 1.5 &&
              !device.Device_View(671).feedback_fresh,
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

void gui_checks()
{
    MaintenanceWindow window;
    window.show();
    QApplication::processEvents();
    auto button = [&](const char *name)
    {
        return window.findChild<QPushButton *>(name);
    };
    check(window.findChild<QProcess *>() == nullptr, "startup opened hardware");
    check(window.findChild<QWidget *>("workspacePages") == nullptr, "old navigation remained");
    check(window.findChild<QWidget *>("demoScenario") == nullptr, "demo scenario remained");
    check(!button("deviceStartSpeed")->isEnabled() && !button("deviceSetSpeed")->isEnabled() &&
              !button("executeAppUpgrade")->isEnabled(),
          "disconnected control enabled");
    check(window.findChild<QLabel *>("deviceUid")->text() == "未知", "invented device identity");
    window.findChild<QLineEdit *>("appPath")->setText("missing.bin");
    button("inspectFiles")->click();
    check(window.findChild<QLabel *>("validationStatus")->text().contains("校验失败"),
          "invalid APP package accepted");
    button("connectDevice")->click();
    check(window.findChild<QProcess *>() == nullptr, "missing SDK opened a device port");
}

void app_package_checks()
{
    QFile fixture(":/fixtures/loader_image_v3.json");
    check(fixture.open(QIODevice::ReadOnly), "APP fixture absent");
    const auto image = QJsonDocument::fromJson(fixture.readAll()).object();
    QTemporaryDir directory;
    check(directory.isValid(), "APP temp directory unavailable");
    const auto image_path = directory.filePath("image.bin");
    const auto manifest_path = directory.filePath("manifest.bin");
    auto write = [](const QString &path, const QByteArray &bytes)
    {
        QFile file(path);
        check(file.open(QIODevice::WriteOnly) && file.write(bytes) == bytes.size(),
              "APP fixture write failed");
    };
    auto bytes = QByteArray::fromHex(image.value("image_hex").toString().toLatin1());
    write(image_path, bytes);
    write(manifest_path, QByteArray::fromHex(image.value("manifest_hex").toString().toLatin1()));
    auto plan = Upgrade_Inspect(image_path);
    check(plan.valid, "valid APP package rejected");
    plan.version += 1;
    QJsonObject report{{"success", true},
                       {"uid", "5800460010504e4831303320"},
                       {"version", double(plan.version)},
                       {"crc32", QString("%1").arg(plan.crc32, 8, 16, QChar('0'))}};
    check(Upgrade_ResultMatches(plan, "5800460010504E4831303320", report),
          "result matching froze APP version or UID");
    report["version"] = double(plan.version - 1);
    check(!Upgrade_ResultMatches(plan, "5800460010504E4831303320", report),
          "wrong package version accepted");
    report["version"] = double(plan.version);
    report["crc32"] = "invalid!";
    check(!Upgrade_ResultMatches(plan, "5800460010504E4831303320", report),
          "invalid image CRC accepted");
    bytes[bytes.size() - 1] ^= 1;
    write(image_path, bytes);
    check(!Upgrade_Inspect(image_path).valid, "damaged APP package accepted");
}

void fake_gui_checks()
{
    QTemporaryDir directory;
    check(directory.isValid(), "fake bench directory unavailable");
    const auto dll = directory.filePath("unused.dll");
    QFile file(dll);
    check(file.open(QIODevice::WriteOnly), "fake SDK marker unavailable");
    file.close();
    MaintenanceWindow window;
    window.show();
    auto button = [&](const char *name)
    {
        return window.findChild<QPushButton *>(name);
    };
    auto *sequence_start = window.findChild<QSpinBox *>("benchSequenceStart");
    check(sequence_start->value() == 0 && sequence_start->maximum() == 65535,
          "bench sequence start changed default/range");
    sequence_start->setValue(10000);
    window.findChild<QLineEdit *>("pythonRuntimePath")
        ->setText(QCoreApplication::applicationFilePath());
    window.findChild<QLineEdit *>("canSdkPath")->setText(dll);
    button("connectDevice")->click();
    auto *bench = window.findChild<QCheckBox *>("deviceBenchAcknowledged");
    check(wait_for(
              [&]
              {
                  return bench->isEnabled();
              },
              1500),
          "fake port did not open");
    check(!sequence_start->isEnabled(), "connected sequence start editable");
    bench->click();
    check(wait_for(
              [&]
              {
                  return button("deviceStartSpeed")->isEnabled();
              },
              1500),
          "fake identity/IDLE did not enable start");
    check(window.findChild<QLabel *>("deviceUid")->text().contains("5700460010504E4831303320"),
          "real reply UID not displayed");
    button("deviceStartSpeed")->click();
    check(wait_for(
              [&]
              {
                  return button("deviceSetSpeed")->isEnabled();
              },
              1500),
          "accepted start did not reach RUNNING");
    window.findChild<QSpinBox *>("deviceSpeedRpm")->setValue(30);
    button("deviceSetSpeed")->click();
    check(wait_for(
              [&]
              {
                  return window.findChild<QLabel *>("deviceControlStatus")
                      ->text()
                      .contains("Type 116");
              },
              1500),
          "116 result did not arrive");
    button("stopMotor")->click();
    auto *status = window.findChild<QLabel *>("deviceControlStatus");
    check(wait_for(
              [&]
              {
                  return button("deviceStartSpeed")->isEnabled() &&
                         status->text().contains("STOP 结果 0/0 且108确认 IDLE");
              },
              2000),
          "one STOP click did not reach 0/0 and IDLE");
    const auto trace = window.findChild<QLabel *>("globalStatus")->toolTip().mid(3);
    QFile events(trace);
    check(events.open(QIODevice::ReadOnly), "fake GUI trace absent");
    int stop_clicks = 0, stop_commands = 0, set_commands = 0;
    int logged_start = -1, first_frame_sequence = -1;
    while (!events.atEnd())
    {
        const auto event = QJsonDocument::fromJson(events.readLine()).object();
        if (event.value("event") == "connect_intent")
        {
            logged_start = event.value("sequence_start").toInt(-1);
        }
        if (event.value("event") == "frame" && first_frame_sequence < 0)
        {
            first_frame_sequence = event.value("sequence").toInt(-1);
        }
        stop_clicks += event.value("event") == "button" && event.value("operation") == "stop";
        stop_commands += event.value("event") == "command_sent" && event.value("type") == 110;
        set_commands += event.value("event") == "command_sent" && event.value("type") == 116;
    }
    check(stop_clicks == 1 && stop_commands == 2 && set_commands == 1,
          "expected 116 and bounded two-request STOP sequence");
    check(logged_start == 10000 && first_frame_sequence == 10001,
          "manual sequence start did not seed first GET");
    button("disconnectDevice")->click();
    check(wait_for(
              [&]
              {
                  return sequence_start->isEnabled();
              },
              1500),
          "sequence start remained locked after disconnect");
    check(window.findChild<QLabel *>("deviceUid")->text() == "未知",
          "disconnected UI retained stale identity");
}

void identity_and_sequence_checks(const QString &settings_path)
{
    const QString uid = "5800460010504E4831303320";
    const auto key = "control-sequences/" + uid + "/source-2";
    QSettings settings(QSettings::IniFormat, QSettings::UserScope, "YG", "YG_MC_HOST");
    settings.setValue(key, 65535);
    settings.sync();
    QTemporaryDir directory;
    const auto dll = directory.filePath("unused.dll");
    QFile marker(dll);
    check(marker.open(QIODevice::WriteOnly), "fake marker creation failed");
    marker.close();
    auto connect_window = [&](MaintenanceWindow &window)
    {
        window.findChild<QLineEdit *>("pythonRuntimePath")
            ->setText(QCoreApplication::applicationFilePath());
        window.findChild<QLineEdit *>("canSdkPath")->setText(dll);
        window.findChild<QPushButton *>("connectDevice")->click();
        check(
            wait_for(
                [&]
                {
                    const auto status = window.findChild<QLabel *>("deviceReadbackStatus")->text();
                    return window.findChild<QLabel *>("deviceUid")->text() != "未知" &&
                           (window.findChild<QCheckBox *>("deviceBenchAcknowledged")->isEnabled() ||
                            status.contains("不兼容") || status.contains("手动起点"));
                },
                1500),
            "fake port did not identify");
        window.findChild<QCheckBox *>("deviceBenchAcknowledged")->click();
    };
    auto start_enabled = [](MaintenanceWindow &window)
    {
        return window.findChild<QPushButton *>("deviceStartSpeed")->isEnabled();
    };
    qputenv("YG_HOST_FAKE_SCENARIO", "second-device");
    {
        MaintenanceWindow window;
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  1500),
              "compatible second UID/new APP version could not start");
        window.findChild<QPushButton *>("deviceStartSpeed")->click();
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QPushButton *>("deviceSetSpeed")->isEnabled();
                  },
                  1500),
              "wrapped start did not reach running");
        settings.sync();
        check(settings.value(key).toUInt() == 2, "65535 control wrap/exit STOP reservation failed");
        window.findChild<QPushButton *>("stopMotor")->click();
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  2000),
              "STOP did not finish");
        window.findChild<QPushButton *>("disconnectDevice")->click();
    }
    settings.sync();
    const auto previous = settings.value(key).toUInt();
    {
        MaintenanceWindow window;
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  1500),
              "automatic saved seed reconnect failed");
        window.findChild<QPushButton *>("deviceStartSpeed")->click();
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QPushButton *>("deviceSetSpeed")->isEnabled();
                  },
                  1500),
              "reconnected start not applied");
        settings.sync();
        check(settings.value(key).toUInt() == previous + 2,
              "reconnect did not use saved high water");
        window.findChild<QPushButton *>("stopMotor")->click();
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  2000),
              "reconnected STOP did not finish");
        window.findChild<QPushButton *>("disconnectDevice")->click();
    }
    for (const auto &scenario : {"wrong-product", "wrong-hardware", "wrong-schema", "wrong-abi"})
    {
        qputenv("YG_HOST_FAKE_SCENARIO", scenario);
        MaintenanceWindow window;
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QLabel *>("deviceReadbackStatus")
                          ->text()
                          .contains("不兼容");
                  },
                  1500),
              "incompatible device not read");
        check(!start_enabled(window), "incompatible product/hardware/schema/ABI enabled power");
        window.findChild<QPushButton *>("disconnectDevice")->click();
    }
    qputenv("YG_HOST_FAKE_SCENARIO", "second-device");
    {
        MaintenanceWindow window;
        window.findChild<QSpinBox *>("benchSequenceStart")->setValue(1);
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QLabel *>("deviceReadbackStatus")
                          ->text()
                          .contains("手动起点");
                  },
                  1500),
              "stale manual seed not rejected");
        check(!start_enabled(window) && window.findChild<QPushButton *>("stopMotor")->isEnabled(),
              "stale seed enabled START or disabled STOP");
        window.findChild<QPushButton *>("disconnectDevice")->click();
    }
    {
        MaintenanceWindow window;
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  1500),
              "storage failure setup failed");
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, dll);
        window.findChild<QPushButton *>("deviceStartSpeed")->click();
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QLabel *>("deviceReadbackStatus")
                          ->text()
                          .contains("无法保存");
                  },
                  1500),
              "unwritable sequence store not blocked");
        check(!start_enabled(window) && window.findChild<QPushButton *>("stopMotor")->isEnabled(),
              "storage failure enabled power or disabled STOP");
        window.findChild<QPushButton *>("disconnectDevice")->click();
        QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settings_path);
    }
    qputenv("YG_HOST_FAKE_SCENARIO", "changed-uid");
    {
        MaintenanceWindow window;
        connect_window(window);
        check(wait_for(
                  [&]
                  {
                      return start_enabled(window);
                  },
                  1500),
              "changed UID setup failed");
        window.findChild<QPushButton *>("deviceStartSpeed")->click();
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QLabel *>("deviceReadbackStatus")
                          ->text()
                          .contains("UID 变化");
                  },
                  4500),
              "session UID change not latched");
        check(!start_enabled(window) &&
                  !window.findChild<QCheckBox *>("deviceBenchAcknowledged")->isChecked(),
              "UID replacement inherited prior target power permission");
        check(wait_for(
                  [&]
                  {
                      return window.findChild<QLabel *>("deviceControlStatus")
                          ->text()
                          .contains("STOP 结果 0/0");
                  },
                  2000) ||
                  window.findChild<QLabel *>("globalStatus")->text().contains("未连接"),
              "UID change did not request STOP");
        window.findChild<QPushButton *>("disconnectDevice")->click();
    }
    qunsetenv("YG_HOST_FAKE_SCENARIO");
}
} // namespace
int main(int argc, char **argv)
{
    if (argc > 1 && QString::fromLocal8Bit(argv[1]) == "-u")
    {
        return fake_reader();
    }
    QApplication app(argc, argv);
    app.setStyle("Fusion");
    QTemporaryDir settings_directory;
    check(settings_directory.isValid(), "isolated settings directory unavailable");
    QSettings::setPath(QSettings::IniFormat, QSettings::UserScope, settings_directory.path());
    try
    {
        device_readback_checks();
        app_package_checks();
        gui_checks();
        fake_gui_checks();
        identity_and_sequence_checks(settings_directory.path());
        std::cout << "host core offline PASS; no hardware access\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
