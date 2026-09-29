#ifndef BOOT_LOADER_ENTRY_H
#define BOOT_LOADER_ENTRY_H
#include <stdbool.h>
#include <stdint.h>
/** v2的boot_epoch为主机生成的非零随机事务nonce；不是设备持久启动代号。 */
typedef struct
{
    uint64_t boot_epoch;
    uint16_t sequence;
    uint8_t source_node;
} BootEntryRequest;
typedef enum
{
    BOOT_ENTRY_UNAVAILABLE,
    BOOT_ENTRY_PENDING,
    BOOT_ENTRY_READY,
    BOOT_ENTRY_DENIED,
    BOOT_ENTRY_FAILED
} BootEntryResult;
/** READY仅代表可发送交接回复，不能被主机/通信层解释为已复位。 */
typedef struct
{
    BootEntryResult result;
    uint64_t ack_ticket;
} BootEntryReply;
/**
 * @brief 向APP唯一生命周期所有者提交或查询进入Loader请求。
 * @param context 组合层注入的所有者，通信层不得直接访问硬件。
 * @param request 精确来源、v2事务nonce和请求序号，调用结束不得保留指针。
 * @param reply 完整值结果，READY必须带非零64位ACK票据。
 * @note 同身份幂等；其他身份不可窃取既有停机事务。PENDING等待STOP、worker归还、
 *       保存空闲及真实关相；未安装支持该布局的Loader必须返回UNAVAILABLE。
 */
typedef void (*BootEntryRequestHandler)(void *context,
                                        const BootEntryRequest *request,
                                        BootEntryReply *reply);
/**
 * @brief 把精确交接回复的物理发送结果交回生命周期所有者。
 * @param context 唯一所有者。
 * @param ticket 对应READY回复携带的内部64位票据，不从线上反序列化。
 * @param success 对应帧物理成功；入队/FIFO空不算成功。
 * @note 前台调用；所有者复核截止、原始身份及关相状态后，才能写ENTER邮箱和复位。
 *       失败必须撤销票据，保持功率禁止；不得自动重新使能。
 */
typedef void (*BootEntryTxHandler)(void *context, uint64_t ticket, bool success);
/**
 * @brief 请求确认当前试运行镜像；所有者复核健康与停机状态。
 * @param context APP组合层唯一所有者。
 * @param version 当前发布版本。
 * @param crc32 当前整包CRC32。
 * @return 已确认或本次持久确认成功返回true；错误身份或不健康返回false。
 * @note 前台调用；不授权功率输出，失败不自动使能。
 */
typedef bool (*BootConfirmHandler)(void *context, uint32_t version, uint32_t crc32);
/** 由APP组合层注入通信适配；request/tx_complete开放56，confirm开放57。 */
typedef struct
{
    void *context;
    BootEntryRequestHandler request;
    BootEntryTxHandler tx_complete;
    BootConfirmHandler confirm;
} BootEntryService;
#endif
