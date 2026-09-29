#pragma once
#include <QStringList>
#include <deque>
#include <optional>
#include <vector>

namespace yg::host
{
enum class DemoScenario
{
    Normal,
    ReadOnly,
    Fault,
    Loader
};
enum class MotionMode
{
    Position,
    Speed,
    Current
};
enum class TaskKind
{
    Sensor,
    ElectricalZero,
    MechanicalZero,
    Cogging,
    Resistance,
    Inductance,
    Inertia,
    Friction
};
enum class TaskPhase
{
    Idle,
    Running,
    Completed,
    Cancelled
};
/** SI 演示样本；参考量可缺失，时间来自模型，不能解释成设备时钟。 */
struct ScopeSample
{
    quint8 demo_node = 0;
    double time_s = 0;
    std::optional<double> position_reference_rad, speed_reference_rad_s, iq_reference_a;
    double position_rad = 0, speed_rad_s = 0, iq_a = 0, voltage_v = 36, temperature_c = 25;
    bool gap_before = false;
};
/** 单次演示任务的候选结果，应用演示模型和设备掉电保存含义不同。 */
struct DemoTask
{
    TaskKind kind = TaskKind::Sensor;
    TaskPhase phase = TaskPhase::Idle;
    quint64 run_id = 0;
    double elapsed_s = 0;
    int progress_percent = 0;
    QString stage, result;
    bool applied_to_demo = false;
};
/** 会话状态只由服务改变，页面切换没有业务副作用。 */
struct SessionState
{
    bool connected = false, unlocked = false, enabled = false, moving = false;
    bool streaming = false, recording = false, fault = false;
    quint8 node = 3;
    DemoScenario scenario = DemoScenario::Normal;
    MotionMode mode = MotionMode::Position;
    double target = 0;
    ScopeSample sample;
    DemoTask task;
    QString message = "未连接";
};
/** 参数所有者确认后的 SI 演示约束，不包含设备参数或传输句柄。 */
struct DemoMotionParameters
{
    double maximum_speed_rad_s = 10, current_limit_a = 3, position_kp_per_s = 1;
};
/** 演示会话唯一所有者；不加载驱动，不接受物理设备操作权。 */
class WorkbenchSession
{
  public:
    /**
     * @brief 连接明确标记的演示节点，连接本身不使能或采集。
     * @param node 当前公司协议节点范围 1..254。
     * @param scenario 演示正常、只读、故障或 Loader 场景。
     * @return 连接完成返回 true；非法节点或活动连接返回 false。
     */
    bool Session_Connect(quint8 node, DemoScenario scenario);
    /** @brief 断开演示连接并停止运动、任务和采集，保留记录。 */
    void Session_Disconnect();
    /**
     * @brief 设置本机演示解锁，不表示真实设备租约。
     * @param unlocked 是否允许演示写操作；上锁同时停止模型运动。
     */
    void Session_Unlock(bool unlocked);
    /**
     * @brief 显式使能或失能演示运动模型。
     * @param enabled true 为使能，false 为停止并失能。
     * @return 条件满足返回 true；只读、故障、Loader 或任务忙返回 false。
     */
    bool Session_Enable(bool enabled);
    /** @brief 停止并失能演示模型，取消当前任务，旧目标不自动恢复。 */
    void Session_Stop();
    /**
     * @brief 显式清除演示故障，不启动模型。
     * @return 正常演示 APP 且已解锁时返回 true。
     */
    bool Session_ClearFault();
    /**
     * @brief 提交一个有限范围的演示目标。
     * @param mode 位置、转速或 Iq 模式。
     * @param target 对应单位 rad、rad/s、A；只作用于本机模型。
     * @return 已使能且可写时返回 true，非法数值或忙返回 false。
     */
    bool Session_Move(MotionMode mode, double target);
    /**
     * @brief 在失能时同步已确认的演示参数，使调参影响模型响应。
     * @param parameters 最大速度 rad/s、Iq 限流 A、位置增益 1/s。
     * @return 有限范围且模型失能时返回 true；拒绝时保留原值。
     */
    bool Session_MotionParameters(const DemoMotionParameters &parameters);
    /**
     * @brief 显式开始或停止 50 Hz 演示遥测。
     * @param active true 开始，false 停止；暂停后恢复形成数据间隙。
     * @return 演示 APP 已连接时返回 true。
     */
    bool Session_Stream(bool active);
    /**
     * @brief 开始新记录或停止当前记录，冻结显示不影响记录。
     * @param active true 开始新的记录，false 停止并保留数据。
     * @return 开始时要求正在采集，容量为 30000 个完整样本。
     */
    bool Session_Record(bool active);
    /**
     * @brief 启动一个独占的标定或辨识演示任务。
     * @param kind 演示任务种类，不能映射成设备命令编号。
     * @return 已连接、已解锁、无故障并失能时返回 true。
     */
    bool Session_StartTask(TaskKind kind);
    /** @brief 取消演示任务；取消终态不能被后续时钟推进改成成功。 */
    void Session_CancelTask();
    /**
     * @brief 审核后记录候选已应用到演示模型，不写设备或参数 Flash。
     * @return 当前任务完成且已解锁时返回 true。
     */
    bool Session_ApplyTask();
    /**
     * @brief 推进确定性演示模型并在服务内采样，每次工作有界。
     * @param seconds 实际间隔秒，有限范围 0..0.2。
     * @note GUI 线程调用，不属于设备或电机调度。
     */
    void Session_Tick(double seconds);
    /**
     * @brief 只读查看演示状态，引用有效到服务销毁。
     * @return 服务持有的状态，调用方不得修改。
     */
    const SessionState &Session_View() const;
    /**
     * @brief 取得有界的 60 秒显示样本，停止采集时仍可观察。
     * @return 服务持有的只读样本，不是设备数据。
     */
    const std::deque<ScopeSample> &Session_Samples() const;
    /**
     * @brief 取得未抽点的完整本次记录。
     * @return 服务持有的只读记录，最多 30000 点。
     */
    const std::vector<ScopeSample> &Session_Recording() const;
    /**
     * @brief 返回最近演示事件，最多 100 条。
     * @return 事件副本，每项明确标记演示来源。
     */
    QStringList Session_Events() const;
    /**
     * @brief 判断演示写操作的公共条件。
     * @return 已连接、已解锁、正常 APP、无故障时返回 true。
     */
    bool Session_CanWrite() const;

  private:
    void event(const QString &message);
    void advance_sample(double seconds);
    SessionState state_;
    DemoMotionParameters parameters_;
    std::deque<ScopeSample> samples_;
    std::vector<ScopeSample> recording_;
    QStringList events_;
    double clock_s_ = 0, sample_accumulator_s_ = 0;
    bool gap_pending_ = true;
    quint64 next_run_id_ = 0;
};
/**
 * @brief 返回本地任务显示名，不分配协议操作码。
 * @param kind 演示任务种类。
 * @return 中文任务名称。
 */
QString Task_Name(TaskKind kind);
} // namespace yg::host
