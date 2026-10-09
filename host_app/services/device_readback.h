#pragma once

#include <QByteArray>
#include <QString>
#include <cstdint>
#include <optional>

namespace yg::host
{
/** 来自已校验公司帧的只读快照；时间由调用方的单调时钟提供。 */
struct DeviceReadbackView
{
    QString uid, sn;
    std::optional<uint32_t> product_id, motor_type, app_version, loader_version, parameter_schema,
        parameter_abi, features;
    std::optional<uint16_t> hardware_revision, supported_modes;
    std::optional<uint8_t> role, boot_state, motor_state, control_mode;
    std::optional<uint64_t> boot_epoch;
    std::optional<uint32_t> fault_bits;
    std::optional<double> position_rad, speed_rad_s, iq_a, bus_voltage_v, mcu_temperature_c;
    bool identity_fresh = false, feedback_fresh = false, stop_confirmed_recently = false;
};

/** 单节点、单请求的线路回读所有者；只解析帧，不打开 SDK 或产生功率命令。 */
class DeviceReadback
{
  public:
    /**
     * @brief 指定目标节点并清除旧请求和快照。
     * @param node 实际 CAN 节点，范围 1..254。
     * @return 节点合法返回 true，否则保持原状态。
     */
    bool Device_Reset(uint8_t node);
    /**
     * @brief 登记外部端口已发出的唯一请求，用于严格关联回复。
     * @param type 只允许 1、2、3、108 或 110；110 只登记 STOP，不在此发送。
     * @param sequence 非零帧序号。
     * @param page Type 1 的 0..2 页；其他类型必须为 0。
     * @param now_ms 单调时间毫秒；回复期限 250 ms。
     * @return 请求可登记返回 true；冲突或非法输入返回 false。
     * @note 110 的结果 0 只证明回复时输出禁止，不能证明转子静止或持续安全。
     */
    bool Device_Begin(uint16_t type, uint16_t sequence, uint16_t page, int64_t now_ms);
    /**
     * @brief 消费一条真实格式的 CAN FD 应用回复，拒绝错节点、序号、CRC、补零与过期。
     * @param can_id 29 位扩展标识符。
     * @param frame CAN FD 数据区，长度必须为合法 DLC；调用方已核对 FD、BRS、EFF 和错误标志。
     * @param now_ms 单调时间毫秒。
     * @return 匹配且语义有效时返回 true；拒绝时不覆盖已确认快照。
     */
    bool Device_Accept(uint32_t can_id, const QByteArray &frame, int64_t now_ms);
    /**
     * @brief 读取带来源期限的只读视图。
     * @param now_ms 单调时间毫秒。
     * @return 身份最长保留 5 s、反馈与 STOP 瞬时确认最长 500 ms；过期位显式为 false。
     */
    DeviceReadbackView Device_View(int64_t now_ms) const;

  private:
    void clear_snapshot();
    struct Pending
    {
        uint16_t type = 0, sequence = 0, page = 0;
        int64_t sent_ms = 0;
    };
    uint8_t node_ = 0;
    std::optional<Pending> pending_;
    DeviceReadbackView view_;
    int64_t page0_ms_ = -1, page1_ms_ = -1, page2_ms_ = -1, caps_ms_ = -1, status_ms_ = -1,
            feedback_ms_ = -1, stop_ms_ = -1;
};
} // namespace yg::host
