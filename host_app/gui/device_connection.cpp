/** CAN FD 读回与固定 SPEED/STOP 控制；所有请求由单一辅助进程顺序发出。 */
#include "maintenance_window.h"
#include <QCheckBox>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QSpinBox>
#include <cmath>

namespace yg::host
{
void MaintenanceWindow::connect_device()
{
    if (device_reader_ || upgrade_process_)
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
           view.app_version == 0x00010300U && view.parameter_schema == 12 &&
           view.parameter_abi == 0x000C10D0U;
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
             view.bus_voltage_v && *view.bus_voltage_v > 0 && *view.bus_voltage_v <= 33.0 &&
             view.bus_current_a && std::abs(*view.bus_current_a) <= 3.0 && view.iq_a &&
             std::abs(*view.iq_a) <= 6.0 && view.speed_rad_s &&
             std::abs(*view.speed_rad_s) <= 31.416 && device_bench_ack_->isChecked() && rpm >= 0 &&
             rpm <= 300)
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

bool MaintenanceWindow::Window_BeginDeviceRead(uint16_t type, uint16_t sequence, uint16_t page)
{
    if (!device_reader_ || type == 110)
    {
        return false;
    }
    return device_readback_.Device_Begin(type, sequence, page, elapsed_.elapsed());
}

bool MaintenanceWindow::Window_ObserveDeviceFrame(uint32_t can_id, const QByteArray &frame)
{
    if (!device_reader_ || !device_readback_.Device_Accept(can_id, frame, elapsed_.elapsed()))
    {
        return false;
    }
    refresh_workbench();
    return true;
}
} // namespace yg::host
