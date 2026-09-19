/**
 * @brief 公司协议 v1.2.3 的离线 C99 编解码示例，只演示消息 105 和 124。
 * @note 所有状态归调用方；不接触 CAN 设备或功率级，不是可直接执行的电机控制器。
 */
#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define SET_POSITION 105U
#define MOTION_FEEDBACK 124U
#define FD_DATA_BYTES 32U
#define PAYLOAD_BYTES 8U
#define LOGICAL_BYTES 26U

/* 元数据由适配层提交给控制器，不能把整个结构体直接发送。 */
typedef struct
{
    uint32_t identifier;
    bool extended;
    bool fd;
    bool brs;
    bool remote;
    uint8_t data_length;
    uint8_t data[64];
} CanFdFrame;

/* 内存对象不作为线格式；有符号测量以明确单位表示。 */
typedef struct
{
    uint16_t type;
    uint16_t frame_sequence;
    uint8_t source;
    uint8_t destination;
    uint8_t command_sequence;
    uint16_t lease_tag;
    int32_t position_mrad;
    int16_t speed_mrad_s;
    int16_t iq_ma;
} MotorMessage;

typedef enum
{
    CODEC_OK,
    CODEC_ARGUMENT,
    CODEC_FORMAT,
    CODEC_LENGTH,
    CODEC_HEADER_CRC,
    CODEC_FRAME_CRC,
    CODEC_ADDRESS,
    CODEC_TYPE,
    CODEC_PADDING,
    CODEC_VALUE
} CodecResult;

static void put_u16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *data, uint32_t value)
{
    for (unsigned i = 0; i < 4U; ++i)
    {
        data[i] = (uint8_t)(value >> (8U * i));
    }
}

static uint16_t get_u16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) | ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/* 显式还原补码，避免超范围无符号转有符号的实现相关行为。 */
static int32_t get_i32(const uint8_t *data)
{
    uint32_t value = get_u32(data);
    return value <= INT32_MAX ? (int32_t)value : -1 - (int32_t)(UINT32_MAX - value);
}

static int16_t get_i16(const uint8_t *data)
{
    uint16_t value = get_u16(data);
    return value <= INT16_MAX ? (int16_t)value : (int16_t)(-1 - (int32_t)(UINT16_MAX - value));
}

/* 公司自定义非反射 CRC；长度仅由本文件固定布局确定。 */
static uint8_t crc8(const uint8_t *data, size_t length)
{
    uint8_t value = 0U;
    for (size_t i = 0; i < length; ++i)
    {
        value ^= data[i];
        for (unsigned bit = 0; bit < 8U; ++bit)
        {
            value = (uint8_t)((value << 1) ^ ((value & 0x80U) != 0U ? 0x9BU : 0U));
        }
    }
    return value;
}

static uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t value = 0xFFFFU;
    for (size_t i = 0; i < length; ++i)
    {
        value ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned bit = 0; bit < 8U; ++bit)
        {
            value = (uint16_t)((value << 1) ^ ((value & 0x8000U) != 0U ? 0xBAADU : 0U));
        }
    }
    return value;
}

static bool addresses_valid(const MotorMessage *message)
{
    if (message->type == SET_POSITION)
    {
        return message->source == 2U && message->destination >= 3U && message->destination <= 7U;
    }
    return message->type == MOTION_FEEDBACK && message->source >= 3U && message->source <= 7U &&
           message->destination == 2U;
}

/* 小端业务编码、公司封装和 CAN 元数据组装；失败不修改输出。 */
static CodecResult pack_message(const MotorMessage *message, CanFdFrame *frame)
{
    if (message == NULL || frame == NULL)
    {
        return CODEC_ARGUMENT;
    }
    if (message->type != SET_POSITION && message->type != MOTION_FEEDBACK)
    {
        return CODEC_TYPE;
    }
    if (!addresses_valid(message))
    {
        return CODEC_ADDRESS;
    }
    if (message->type == SET_POSITION && message->position_mrad == INT32_MIN)
    {
        return CODEC_VALUE;
    }
    CanFdFrame encoded = {0};
    uint8_t *data = encoded.data;
    uint32_t priority = message->type == SET_POSITION ? 2U : 3U;
    encoded.identifier =
        (priority << 26) | (0xEFU << 16) | ((uint32_t)message->destination << 8) | message->source;
    encoded.extended = true;
    encoded.fd = true;
    encoded.brs = true;
    encoded.data_length = FD_DATA_BYTES;
    put_u16(data, 0xA55AU);
    data[2] = 1U;
    data[4] = message->source;
    data[5] = message->destination;
    put_u16(data + 6, message->type);
    put_u16(data + 8, message->frame_sequence);
    put_u32(data + 10, PAYLOAD_BYTES);
    data[15] = crc8(data, 15U);
    if (message->type == SET_POSITION)
    {
        data[16] = SET_POSITION;
        data[17] = message->command_sequence;
        put_u16(data + 18, message->lease_tag);
        put_u32(data + 20, (uint32_t)message->position_mrad);
    }
    else
    {
        put_u32(data + 16, (uint32_t)message->position_mrad);
        put_u16(data + 20, (uint16_t)message->speed_mrad_s);
        put_u16(data + 22, (uint16_t)message->iq_ma);
    }
    put_u16(data + 24, crc16(data, 24U));
    *frame = encoded;
    return CODEC_OK;
}

/* 模拟控制器已验过硬件 CRC 后的接收路径；不解引用未经验证的 len。 */
static CodecResult
unpack_message(const CanFdFrame *frame, uint8_t local_node, MotorMessage *message)
{
    if (frame == NULL || message == NULL)
    {
        return CODEC_ARGUMENT;
    }
    if (!frame->extended || !frame->fd || !frame->brs || frame->remote ||
        frame->identifier > 0x1FFFFFFFU || (frame->identifier & 0x03FF0000U) != 0x00EF0000U)
    {
        return CODEC_FORMAT;
    }
    if (frame->data_length != FD_DATA_BYTES)
    {
        return CODEC_LENGTH;
    }
    const uint8_t *data = frame->data;
    if (get_u16(data) != 0xA55AU || data[2] != 1U || data[3] != 0U || data[14] != 0U)
    {
        return CODEC_FORMAT;
    }
    if (crc8(data, 15U) != data[15])
    {
        return CODEC_HEADER_CRC;
    }
    if (get_u32(data + 10) != PAYLOAD_BYTES)
    {
        return CODEC_LENGTH;
    }
    if (crc16(data, 24U) != get_u16(data + 24))
    {
        return CODEC_FRAME_CRC;
    }
    for (size_t i = LOGICAL_BYTES; i < FD_DATA_BYTES; ++i)
    {
        if (data[i] != 0U)
        {
            return CODEC_PADDING;
        }
    }
    MotorMessage decoded = {0};
    decoded.type = get_u16(data + 6);
    decoded.frame_sequence = get_u16(data + 8);
    decoded.source = data[4];
    decoded.destination = data[5];
    if (decoded.type != SET_POSITION && decoded.type != MOTION_FEEDBACK)
    {
        return CODEC_TYPE;
    }
    if (!addresses_valid(&decoded) || decoded.destination != local_node ||
        (uint8_t)frame->identifier != decoded.source ||
        (uint8_t)(frame->identifier >> 8) != decoded.destination)
    {
        return CODEC_ADDRESS;
    }
    uint32_t priority = decoded.type == SET_POSITION ? 2U : 3U;
    if ((frame->identifier >> 26) != priority)
    {
        return CODEC_TYPE;
    }
    if (decoded.type == SET_POSITION)
    {
        if (data[16] != SET_POSITION)
        {
            return CODEC_TYPE;
        }
        decoded.command_sequence = data[17];
        decoded.lease_tag = get_u16(data + 18);
        decoded.position_mrad = get_i32(data + 20);
        if (decoded.position_mrad == INT32_MIN)
        {
            return CODEC_VALUE;
        }
    }
    else
    {
        decoded.position_mrad = get_i32(data + 16);
        decoded.speed_mrad_s = get_i16(data + 20);
        decoded.iq_ma = get_i16(data + 22);
    }
    *message = decoded;
    return CODEC_OK;
}

static void print_frame(const char *label, const CanFdFrame *frame)
{
    printf("%s ID=%08" PRIX32 " IDE=1 FDF=1 BRS=1 DLC=13 LEN=%u DATA=",
           label,
           frame->identifier,
           (unsigned)frame->data_length);
    for (size_t i = 0; i < frame->data_length; ++i)
    {
        printf("%02X%s", frame->data[i], i + 1U == frame->data_length ? "\n" : " ");
    }
}

/* 只用于离线自测，绝不替代真实硬件状态检查。 */
#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition);                           \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

/**
 * @brief 在内存中演示主机发位置目标、电机解包、模拟测量回传和主机解包。
 * @return 全部离线检查成功返回 0，否则返回 1。
 * @note 不打开任何硬件接口；printf 仅在主机进程执行。
 */
int main(void)
{
    MotorMessage command = {.type = SET_POSITION,
                            .frame_sequence = 2U,
                            .source = 2U,
                            .destination = 3U,
                            .command_sequence = 2U,
                            .lease_tag = 0x1234U,
                            .position_mrad = 1000};
    CanFdFrame tx;
    MotorMessage received;
    CHECK(pack_message(&command, &tx) == CODEC_OK);
    print_frame("CONTROL", &tx);
    /* 此内存传递替代 comm_hw 收发；不会触发位置环或自动使能。 */
    CHECK(unpack_message(&tx, 3U, &received) == CODEC_OK);
    CHECK(received.position_mrad == 1000 && received.lease_tag == 0x1234U &&
          received.command_sequence == 2U && received.frame_sequence == 2U);
    printf("DECODE_CONTROL position_mrad=%" PRId32 " lease=%04X command_seq=%u\n",
           received.position_mrad,
           received.lease_tag,
           received.command_sequence);

    MotorMessage snapshot = {.type = MOTION_FEEDBACK,
                             .frame_sequence = 10U,
                             .source = 3U,
                             .destination = 2U,
                             .position_mrad = 950,
                             .speed_mrad_s = -1200,
                             .iq_ma = 300};
    CanFdFrame feedback;
    CHECK(pack_message(&snapshot, &feedback) == CODEC_OK);
    print_frame("FEEDBACK", &feedback);
    CHECK(unpack_message(&feedback, 2U, &received) == CODEC_OK);
    CHECK(received.position_mrad == 950 && received.speed_mrad_s == -1200 && received.iq_ma == 300);
    printf("DECODE_FEEDBACK position_mrad=%" PRId32 " speed_mrad_s=%d iq_ma=%d\n",
           received.position_mrad,
           received.speed_mrad_s,
           received.iq_ma);

    const uint8_t check[] = "123456789";
    CHECK(crc8(check, 9U) == 0xEAU && crc16(check, 9U) == 0x932DU);
    CHECK(unpack_message(&tx, 4U, &received) == CODEC_ADDRESS);
    CHECK(pack_message(NULL, &tx) == CODEC_ARGUMENT);
    CHECK(unpack_message(NULL, 3U, &received) == CODEC_ARGUMENT);
    /* 任意数据位破坏必须失败，且不能向业务层发布部分对象。 */
    for (size_t i = 0; i < FD_DATA_BYTES; ++i)
    {
        for (unsigned bit = 0; bit < 8U; ++bit)
        {
            CanFdFrame bad = tx;
            MotorMessage before = received;
            bad.data[i] ^= (uint8_t)(1U << bit);
            CHECK(unpack_message(&bad, 3U, &received) != CODEC_OK);
            CHECK(memcmp(&before, &received, sizeof(received)) == 0);
        }
    }
    for (unsigned length = 0; length <= 65U; ++length)
    {
        CanFdFrame bad = tx;
        bad.data_length = (uint8_t)length;
        CHECK(unpack_message(&bad, 3U, &received) ==
              (length == FD_DATA_BYTES ? CODEC_OK : CODEC_LENGTH));
    }
    CanFdFrame bad = tx;
    bad.identifier ^= 1U;
    CHECK(unpack_message(&bad, 3U, &received) == CODEC_ADDRESS);
    bad = tx;
    bad.identifier |= 0x20000000U;
    CHECK(unpack_message(&bad, 3U, &received) == CODEC_FORMAT);
    bad = tx;
    bad.brs = false;
    CHECK(unpack_message(&bad, 3U, &received) == CODEC_FORMAT);
    bad = tx;
    bad.data[16] = 104U;
    put_u16(bad.data + 24, crc16(bad.data, 24U));
    CHECK(unpack_message(&bad, 3U, &received) == CODEC_TYPE);
    command.position_mrad = -1000;
    CHECK(pack_message(&command, &tx) == CODEC_OK);
    CHECK(unpack_message(&tx, 3U, &received) == CODEC_OK && received.position_mrad == -1000);
    command.position_mrad = INT32_MIN;
    CHECK(pack_message(&command, &tx) == CODEC_VALUE);
    puts("PASS offline codec checks; no hardware accessed");
    return 0;
}
