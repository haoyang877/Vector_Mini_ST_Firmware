/** 只读帧按共享统一帧核心校验；主机仅解释已冻结的业务 payload。 */
#include "device_readback.h"
extern "C"
{
#include "shared/protocol/yg_protocol_byte_order.h"
#include "shared/protocol/yg_protocol_can_id.h"
#include "shared/protocol/yg_protocol_frame_codec.h"
}
#include <algorithm>
#include <limits>

namespace yg::host
{
namespace
{
bool fresh(int64_t stamp, int64_t now_ms, int64_t lifetime_ms)
{
    return stamp >= 0 && now_ms >= stamp && now_ms - stamp <= lifetime_ms;
}

std::optional<double>
signed_measurement(const uint8_t *data, uint16_t bits, unsigned index, bool wide, double divisor)
{
    const int32_t value =
        wide ? int32_t(YgProtocol_ReadU32Le(data)) : int16_t(YgProtocol_ReadU16Le(data));
    const int32_t unavailable =
        wide ? std::numeric_limits<int32_t>::min() : std::numeric_limits<int16_t>::min();
    if ((bits & (1U << index)) == 0U)
    {
        return std::nullopt;
    }
    if (value == unavailable)
    {
        return std::nullopt;
    }
    return double(value) / divisor;
}

bool valid_measurement(const uint8_t *data, uint16_t bits, unsigned index, bool wide)
{
    const int32_t value =
        wide ? int32_t(YgProtocol_ReadU32Le(data)) : int16_t(YgProtocol_ReadU16Le(data));
    const int32_t unavailable =
        wide ? std::numeric_limits<int32_t>::min() : std::numeric_limits<int16_t>::min();
    return ((bits & (1U << index)) != 0U) == (value != unavailable);
}

bool valid_state(uint8_t state)
{
    return state == 2 || state == 3 || (state >= 4 && state <= 9);
}

int expected_dlc(uint32_t logical_size)
{
    for (const int length : {20, 24, 32, 48, 64})
    {
        if (logical_size <= uint32_t(length))
        {
            return length;
        }
    }
    return 0;
}
} // namespace

bool Device_SupportsSpeed(const DeviceReadbackView &view)
{
    constexpr uint32_t required_features = (1U << 8U) | (1U << 10U) | (1U << 11U) | (1U << 12U);
    return view.identity_fresh &&
           (view.features.value_or(0) & required_features) == required_features &&
           (view.supported_modes.value_or(0) & (1U << 2U)) != 0;
}

bool DeviceReadback::Device_Reset(uint8_t node)
{
    if (node == 0 || node == 255)
    {
        return false;
    }
    node_ = node;
    pending_.reset();
    clear_snapshot();
    return true;
}

void DeviceReadback::clear_snapshot()
{
    view_ = {};
    page0_ms_ = page1_ms_ = page2_ms_ = caps_ms_ = status_ms_ = feedback_ms_ = stop_ms_ = -1;
}

bool DeviceReadback::Device_Begin(uint16_t type, uint16_t sequence, uint16_t page, int64_t now_ms)
{
    if (node_ == 0 || sequence == 0 || now_ms < 0 ||
        (type != 1 && type != 2 && type != 3 && type != 108 && type != 110) ||
        (type == 1 ? page > 2 : page != 0) || (pending_ && fresh(pending_->sent_ms, now_ms, 250)))
    {
        return false;
    }
    pending_ = Pending{type, sequence, page, now_ms};
    if (type == 110)
    {
        stop_ms_ = -1;
    }
    return true;
}

bool DeviceReadback::Device_Accept(uint32_t can_id, const QByteArray &frame, int64_t now_ms)
{
    if (!pending_ || !fresh(pending_->sent_ms, now_ms, 250))
    {
        return false;
    }
    const int size = frame.size();
    if (size != 20 && size != 24 && size != 32 && size != 48 && size != 64)
    {
        return false;
    }
    yg_protocol_can_id_fields_t address{};
    if (yg_protocol_can_id_decode(can_id, &address) != YG_PROTOCOL_OK || address.priority != 3 ||
        address.source_node != node_ || address.destination_node != 2)
    {
        return false;
    }
    const auto *bytes = reinterpret_cast<const uint8_t *>(frame.constData());
    if (size < 18)
    {
        return false;
    }
    const uint32_t payload_size = YgProtocol_ReadU32Le(bytes + 10);
    if (payload_size > 46 || expected_dlc(18U + payload_size) != size ||
        !std::all_of(bytes + 18 + payload_size,
                     bytes + size,
                     [](uint8_t value)
                     {
                         return value == 0;
                     }))
    {
        return false;
    }
    yg_protocol_frame_view_t message{};
    if (yg_protocol_frame_decode(bytes, 18U + payload_size, &message) != YG_PROTOCOL_OK ||
        message.flags != YG_PROTOCOL_FLAGS_RESPONSE || message.reserved != 0 ||
        message.source_node != node_ || message.destination_node != 2 ||
        message.message_type != pending_->type || message.sequence != pending_->sequence)
    {
        return false;
    }
    const auto *data = message.payload;
    if (message.message_type == 1 && message.payload_length == 32)
    {
        if (pending_->page == 0)
        {
            if (std::all_of(data,
                            data + 12,
                            [](uint8_t value)
                            {
                                return value == 0;
                            }))
            {
                return false;
            }
            const auto uid = QString::fromLatin1(
                QByteArray(reinterpret_cast<const char *>(data), 12).toHex().toUpper());
            if (!view_.uid.isEmpty() && view_.uid != uid)
            {
                clear_snapshot();
            }
            view_.uid = uid;
            view_.product_id = YgProtocol_ReadU32Le(data + 12);
            view_.hardware_revision = YgProtocol_ReadU16Le(data + 16);
            view_.app_version = YgProtocol_ReadU32Le(data + 20);
            view_.loader_version = YgProtocol_ReadU32Le(data + 24);
            page0_ms_ = now_ms;
        }
        else if (pending_->page == 1)
        {
            if (data[21] != node_ || data[20] > 1 || data[22] < 1 || data[22] > 2 || data[23] > 3)
            {
                return false;
            }
            const auto sn_end = std::find(data, data + 16, 0);
            if (data[20] && (sn_end == data ||
                             !std::all_of(data,
                                          sn_end,
                                          [](uint8_t value)
                                          {
                                              return (value >= '0' && value <= '9') ||
                                                     (value >= 'A' && value <= 'Z') ||
                                                     (value >= 'a' && value <= 'z') || value == '-';
                                          }) ||
                             !std::all_of(sn_end,
                                          data + 16,
                                          [](uint8_t value)
                                          {
                                              return value == 0;
                                          })))
            {
                return false;
            }
            view_.sn = data[20] ? QString::fromLatin1(reinterpret_cast<const char *>(data),
                                                      int(sn_end - data))
                                : QString();
            view_.motor_type =
                data[20] ? std::optional<uint32_t>(YgProtocol_ReadU32Le(data + 16)) : std::nullopt;
            view_.role = data[22];
            view_.boot_state = data[23];
            page1_ms_ = now_ms;
        }
        else
        {
            view_.parameter_abi = YgProtocol_ReadU32Le(data + 8);
            view_.parameter_schema = YgProtocol_ReadU32Le(data + 12);
            page2_ms_ = now_ms;
        }
    }
    else if (message.message_type == 2 && message.payload_length == 16)
    {
        if (YgProtocol_ReadU16Le(data + 6) != 64)
        {
            return false;
        }
        view_.features = YgProtocol_ReadU32Le(data);
        view_.supported_modes = YgProtocol_ReadU16Le(data + 4);
        caps_ms_ = now_ms;
    }
    else if (message.message_type == 3 && message.payload_length == 16)
    {
        const uint16_t result = YgProtocol_ReadU16Le(data);
        const uint16_t valid = YgProtocol_ReadU16Le(data + 6);
        const uint64_t epoch = YgProtocol_ReadU64Le(data + 8);
        if ((result != 0 && result != 6) || YgProtocol_ReadU16Le(data + 2) != 0 || data[4] < 1 ||
            data[4] > 2 || data[5] > 3 || valid > 1 || (valid == 1) != (result == 0) ||
            (valid == 1) != (epoch != 0) || (data[4] == 1 && data[5] == 3) ||
            (data[4] == 2 && data[5] != 3) || (data[4] == 1 && valid == 0 && data[5] != 0))
        {
            return false;
        }
        if (view_.boot_epoch != (valid ? std::optional<uint64_t>(epoch) : std::nullopt) &&
            (status_ms_ >= 0 || page0_ms_ >= 0 || page1_ms_ >= 0 || page2_ms_ >= 0 ||
             caps_ms_ >= 0 || feedback_ms_ >= 0 || stop_ms_ >= 0))
        {
            clear_snapshot();
        }
        view_.role = data[4];
        view_.boot_state = data[5];
        view_.boot_epoch = valid ? std::optional<uint64_t>(epoch) : std::nullopt;
        status_ms_ = now_ms;
    }
    else if (message.message_type == 108 && message.payload_length == 45)
    {
        const uint16_t bits = YgProtocol_ReadU16Le(data + 24);
        if (YgProtocol_ReadU16Le(data) != 0 || YgProtocol_ReadU16Le(data + 2) != 0 ||
            (bits & 0xF000U) != 0 || data[42] != node_ || !valid_state(data[43]) ||
            !valid_measurement(data + 8, bits, 0, true) ||
            !valid_measurement(data + 12, bits, 1, true) ||
            !valid_measurement(data + 16, bits, 2, true) ||
            !valid_measurement(data + 20, bits, 3, true) ||
            !valid_measurement(data + 26, bits, 4, false) ||
            !valid_measurement(data + 28, bits, 5, false) ||
            !valid_measurement(data + 30, bits, 6, false) ||
            !valid_measurement(data + 32, bits, 7, false) ||
            !valid_measurement(data + 34, bits, 8, false) ||
            !valid_measurement(data + 36, bits, 9, false) ||
            !valid_measurement(data + 38, bits, 10, false) ||
            !valid_measurement(data + 40, bits, 11, false))
        {
            return false;
        }
        view_.fault_bits = YgProtocol_ReadU32Le(data + 4);
        view_.position_rad = signed_measurement(data + 12, bits, 1, true, 1000);
        view_.speed_rad_s = signed_measurement(data + 20, bits, 3, true, 1000);
        view_.iq_a = signed_measurement(data + 32, bits, 7, false, 1000);
        view_.bus_voltage_v = signed_measurement(data + 26, bits, 4, false, 100);
        view_.bus_current_a = signed_measurement(data + 28, bits, 5, false, 1000);
        view_.mcu_temperature_c = signed_measurement(data + 34, bits, 8, false, 100);
        view_.motor_state = data[43];
        view_.control_mode = data[44];
        // 后续状态帧不证明输出持续关闭；STOP 回执只保留为瞬时历史事实。
        stop_ms_ = -1;
        feedback_ms_ = now_ms;
    }
    else if (message.message_type == 110 && message.payload_length == 4)
    {
        const uint16_t result = YgProtocol_ReadU16Le(data);
        if ((result != 0 && result != 1) || YgProtocol_ReadU16Le(data + 2) != 0)
        {
            return false;
        }
        stop_ms_ = result == 0 ? now_ms : -1;
    }
    else
    {
        return false;
    }
    pending_.reset();
    return true;
}

DeviceReadbackView DeviceReadback::Device_View(int64_t now_ms) const
{
    auto result = view_;
    result.identity_fresh = fresh(page0_ms_, now_ms, 5000) && fresh(page1_ms_, now_ms, 5000) &&
                            fresh(caps_ms_, now_ms, 5000);
    result.feedback_fresh = fresh(feedback_ms_, now_ms, 500);
    result.stop_confirmed_recently = fresh(stop_ms_, now_ms, 500);
    if (!fresh(page0_ms_, now_ms, 5000))
    {
        result.uid.clear();
        result.product_id.reset();
        result.app_version.reset();
        result.loader_version.reset();
        result.hardware_revision.reset();
    }
    if (!fresh(page1_ms_, now_ms, 5000))
    {
        result.sn.clear();
        result.motor_type.reset();
        if (!fresh(status_ms_, now_ms, 5000))
        {
            result.role.reset();
            result.boot_state.reset();
        }
    }
    if (!fresh(caps_ms_, now_ms, 5000))
    {
        result.features.reset();
        result.supported_modes.reset();
    }
    if (!fresh(page2_ms_, now_ms, 5000))
    {
        result.parameter_abi.reset();
        result.parameter_schema.reset();
    }
    if (!fresh(status_ms_, now_ms, 5000))
    {
        result.boot_epoch.reset();
    }
    if (!result.feedback_fresh)
    {
        result.motor_state.reset();
        result.control_mode.reset();
        result.fault_bits.reset();
        result.position_rad.reset();
        result.speed_rad_s.reset();
        result.iq_a.reset();
        result.bus_voltage_v.reset();
        result.bus_current_a.reset();
        result.mcu_temperature_c.reset();
    }
    return result;
}
} // namespace yg::host
