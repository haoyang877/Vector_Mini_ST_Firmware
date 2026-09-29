/** 演示会话保持有界记录和显式操作门禁，物理传输不属于此模型。 */
#include "workbench_session.h"
#include <algorithm>
#include <cmath>

namespace yg::host
{
QString Task_Name(TaskKind kind)
{
    switch (kind)
    {
    case TaskKind::Sensor:
        return "角度传感器标定";
    case TaskKind::ElectricalZero:
        return "电角度零位标定";
    case TaskKind::MechanicalZero:
        return "机械参考零位";
    case TaskKind::Cogging:
        return "齿槽转矩标定";
    case TaskKind::Resistance:
        return "电阻辨识";
    case TaskKind::Inductance:
        return "电感辨识";
    case TaskKind::Inertia:
        return "转动惯量辨识";
    case TaskKind::Friction:
        return "摩擦力辨识";
    }
    return "未知任务";
}

void WorkbenchSession::event(const QString &message)
{
    state_.message = message;
    events_.append(QString("[离线演示 %1 s] %2").arg(clock_s_, 0, 'f', 2).arg(message));
    if (events_.size() > 100)
    {
        events_.removeFirst();
    }
}

bool WorkbenchSession::Session_Connect(quint8 node, DemoScenario scenario)
{
    if (node == 0 || node == 255 || state_.connected)
    {
        return false;
    }
    state_ = {};
    parameters_ = {};
    state_.connected = true;
    state_.node = node;
    state_.sample.demo_node = node;
    state_.scenario = scenario;
    state_.fault = scenario == DemoScenario::Fault;
    samples_.clear();
    clock_s_ = 0;
    sample_accumulator_s_ = 0;
    gap_pending_ = true;
    event("已连接演示节点；未使能、未启动采集");
    return true;
}

void WorkbenchSession::Session_Disconnect()
{
    Session_Stop();
    state_.connected = false;
    state_.unlocked = false;
    state_.streaming = false;
    state_.recording = false;
    gap_pending_ = true;
    event("演示连接已断开，记录保留");
}

bool WorkbenchSession::Session_CanWrite() const
{
    return state_.connected && state_.unlocked && !state_.fault &&
           state_.scenario != DemoScenario::ReadOnly && state_.scenario != DemoScenario::Loader;
}

void WorkbenchSession::Session_Unlock(bool unlocked)
{
    state_.unlocked = state_.connected && unlocked;
    if (!state_.unlocked)
    {
        Session_Stop();
    }
    event(state_.unlocked ? "本机演示操作已解锁" : "本机演示操作已锁定");
}

bool WorkbenchSession::Session_Enable(bool enabled)
{
    if (!enabled)
    {
        Session_Stop();
        return true;
    }
    if (!Session_CanWrite() || state_.task.phase == TaskPhase::Running)
    {
        return false;
    }
    state_.enabled = true;
    event("演示模型已使能，尚未提交运动目标");
    return true;
}

void WorkbenchSession::Session_Stop()
{
    state_.enabled = false;
    state_.moving = false;
    state_.target = 0;
    state_.sample.speed_rad_s = 0;
    state_.sample.iq_a = 0;
    state_.sample.position_reference_rad.reset();
    state_.sample.speed_reference_rad_s.reset();
    state_.sample.iq_reference_a.reset();
    Session_CancelTask();
    event("演示模型已停止并失能，旧目标已清除");
}

bool WorkbenchSession::Session_ClearFault()
{
    if (!state_.connected || !state_.unlocked || state_.scenario == DemoScenario::ReadOnly ||
        state_.scenario == DemoScenario::Loader)
    {
        return false;
    }
    state_.fault = false;
    event("演示故障已清除；不会自动使能或运动");
    return true;
}

bool WorkbenchSession::Session_Move(MotionMode mode, double target)
{
    const auto limit = mode == MotionMode::Current ? parameters_.current_limit_a
                       : mode == MotionMode::Speed ? parameters_.maximum_speed_rad_s
                                                   : 100.0;
    if (!Session_CanWrite() || !state_.enabled || state_.task.phase == TaskPhase::Running ||
        !std::isfinite(target) || std::abs(target) > limit ||
        (mode != MotionMode::Position && mode != MotionMode::Speed && mode != MotionMode::Current))
    {
        return false;
    }
    state_.mode = mode;
    state_.target = target;
    state_.moving = true;
    event("已提交演示运动目标");
    return true;
}

bool WorkbenchSession::Session_MotionParameters(const DemoMotionParameters &parameters)
{
    if (state_.enabled || state_.task.phase == TaskPhase::Running ||
        !std::isfinite(parameters.maximum_speed_rad_s) || parameters.maximum_speed_rad_s <= 0 ||
        parameters.maximum_speed_rad_s > 10000 || !std::isfinite(parameters.current_limit_a) ||
        parameters.current_limit_a <= 0 || parameters.current_limit_a > 10000 ||
        !std::isfinite(parameters.position_kp_per_s) || parameters.position_kp_per_s < 0 ||
        parameters.position_kp_per_s > 10)
    {
        return false;
    }
    parameters_ = parameters;
    return true;
}

bool WorkbenchSession::Session_Stream(bool active)
{
    if (active && (!state_.connected || state_.scenario == DemoScenario::Loader))
    {
        return false;
    }
    state_.streaming = active;
    if (!active)
    {
        state_.recording = false;
        gap_pending_ = true;
    }
    event(active ? "演示遥测已开始 · 50 Hz" : "演示遥测已停止，现有曲线和记录保留");
    return true;
}

bool WorkbenchSession::Session_Record(bool active)
{
    if (active && (!state_.connected || !state_.streaming))
    {
        return false;
    }
    if (active)
    {
        recording_.clear();
    }
    state_.recording = active;
    event(active ? "新演示记录已开始" : "演示记录已停止");
    return true;
}

bool WorkbenchSession::Session_StartTask(TaskKind kind)
{
    if (!Session_CanWrite() || state_.enabled || state_.task.phase == TaskPhase::Running ||
        kind < TaskKind::Sensor || kind > TaskKind::Friction)
    {
        return false;
    }
    state_.task = {};
    state_.task.kind = kind;
    state_.task.run_id = ++next_run_id_;
    state_.task.phase = TaskPhase::Running;
    state_.task.stage = "演示条件检查";
    event("已开始 " + Task_Name(kind) + "（演示）");
    return true;
}

void WorkbenchSession::Session_CancelTask()
{
    if (state_.task.phase == TaskPhase::Running)
    {
        state_.task.phase = TaskPhase::Cancelled;
        state_.task.stage = "已取消（演示）";
        state_.task.result.clear();
        event("演示任务已取消，没有完成结果");
    }
}

bool WorkbenchSession::Session_ApplyTask()
{
    if (!Session_CanWrite() || state_.enabled || state_.task.phase != TaskPhase::Completed)
    {
        return false;
    }
    state_.task.applied_to_demo = true;
    event("候选已标记应用到演示模型；没有设备参数写入或掉电保存");
    return true;
}

void WorkbenchSession::advance_sample(double seconds)
{
    auto &sample = state_.sample;
    sample.time_s = clock_s_;
    if (!state_.moving)
    {
        return;
    }
    const auto speed_rad_s =
        state_.mode == MotionMode::Position
            ? std::clamp((state_.target - sample.position_rad) * parameters_.position_kp_per_s,
                         -parameters_.maximum_speed_rad_s,
                         parameters_.maximum_speed_rad_s)
        : state_.mode == MotionMode::Speed ? state_.target
                                           : std::clamp(state_.target * 2.0,
                                                        -parameters_.maximum_speed_rad_s,
                                                        parameters_.maximum_speed_rad_s);
    sample.speed_rad_s += (speed_rad_s - sample.speed_rad_s) * std::min(1.0, seconds * 12.0);
    sample.position_rad += sample.speed_rad_s * seconds;
    sample.iq_a = state_.mode == MotionMode::Current
                      ? state_.target
                      : std::clamp((speed_rad_s - sample.speed_rad_s) * 0.2,
                                   -parameters_.current_limit_a,
                                   parameters_.current_limit_a);
    sample.position_reference_rad =
        state_.mode == MotionMode::Position ? std::optional<double>(state_.target) : std::nullopt;
    sample.speed_reference_rad_s = speed_rad_s;
    sample.iq_reference_a =
        state_.mode == MotionMode::Current ? std::optional<double>(state_.target) : std::nullopt;
}

void WorkbenchSession::Session_Tick(double seconds)
{
    if (!std::isfinite(seconds) || seconds < 0 || seconds > 0.2 || !state_.connected)
    {
        return;
    }
    clock_s_ += seconds;
    auto &task = state_.task;
    if (task.phase == TaskPhase::Running)
    {
        task.elapsed_s += seconds;
        task.progress_percent = std::min(100, int(task.elapsed_s * 25));
        task.stage = task.elapsed_s < 0.5   ? "演示条件检查"
                     : task.elapsed_s < 3.5 ? "演示采样"
                                            : "演示验证";
        if (task.elapsed_s >= 4)
        {
            task.phase = TaskPhase::Completed;
            task.stage = "演示完成 · 候选待审核";
            switch (task.kind)
            {
            case TaskKind::Sensor:
                task.result = "示例方向 +1；角度偏置 0.012 rad；误差 0.002 rad";
                break;
            case TaskKind::ElectricalZero:
                task.result = "示例电角度偏置 0.035 rad；不是机械零点";
                break;
            case TaskKind::MechanicalZero:
                task.result = "示例机械参考偏置 0.010 rad";
                break;
            case TaskKind::Cogging:
                task.result = "示例 Iq 补偿幅值 0.025 A；没有真实补偿表";
                break;
            case TaskKind::Resistance:
                task.result = "示例相电阻 Rs 0.12 Ω；模型温度 25 °C";
                break;
            case TaskKind::Inductance:
                task.result = "示例 Ld 0.00018 H；Lq 0.00022 H";
                break;
            case TaskKind::Inertia:
                task.result = "示例电机轴含负载等效 J 0.00025 kg·m²";
                break;
            case TaskKind::Friction:
                task.result = "示例 Ic 0.08 A；B 0.002 A/(rad/s)；无可信 Kt，未换算转矩";
                break;
            }
            event(Task_Name(task.kind) + "演示完成；未应用、未保存设备");
        }
    }
    sample_accumulator_s_ += seconds;
    // 单次间隔上限 0.2 s，最多推进十个 20 ms 模型样本。
    while (sample_accumulator_s_ + 1e-12 >= 0.02)
    {
        sample_accumulator_s_ = std::max(0.0, sample_accumulator_s_ - 0.02);
        advance_sample(0.02);
        state_.sample.time_s = clock_s_ - sample_accumulator_s_;
        if (!state_.streaming)
        {
            continue;
        }
        auto sample = state_.sample;
        sample.gap_before = gap_pending_;
        gap_pending_ = false;
        samples_.push_back(sample);
        if (samples_.size() > 3000)
        {
            samples_.pop_front();
        }
        if (state_.recording)
        {
            recording_.push_back(sample);
            if (recording_.size() >= 30000)
            {
                state_.recording = false;
                event("演示记录已达 30000 点，停止记录并保留数据");
            }
        }
    }
}

const SessionState &WorkbenchSession::Session_View() const
{
    return state_;
}
const std::deque<ScopeSample> &WorkbenchSession::Session_Samples() const
{
    return samples_;
}
const std::vector<ScopeSample> &WorkbenchSession::Session_Recording() const
{
    return recording_;
}
QStringList WorkbenchSession::Session_Events() const
{
    return events_;
}
} // namespace yg::host
