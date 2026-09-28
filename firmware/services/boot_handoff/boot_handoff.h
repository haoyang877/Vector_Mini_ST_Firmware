#ifndef BOOT_HANDOFF_H
#define BOOT_HANDOFF_H

#include <stdbool.h>
#include <stdint.h>

#include "loader_entry.h"

/** 当前健康值；进展计数不能替代健康资格，健康资格也不能替代持续进展。 */
typedef struct
{
    uint32_t supervisor_epoch;
    bool healthy;
    bool confirmation_ready;
} BootHandoffHealth;

/** APP组合层的实际状态所有者接点，服务不导入APP或电机对象。 */
typedef struct
{
    /** @brief 保留原票据并撤销目标。 @param context 所有者。 @param ticket 非零票据。
     * @return 所有者已受理返回true；不表示关相完成。 */
    bool (*request_stop)(void *context, uint64_t ticket);
    /** @brief 复核停机、资源和保存均空闲。 @param context 所有者。 @param ticket 原票据。
     * @return 全部真实条件满足返回true。 */
    bool (*stop_ready)(void *context, uint64_t ticket);
    /** @brief 读取调度、通信及确认健康。 @param context 所有者。 @param health 值输出。 */
    void (*read_health)(void *context, BootHandoffHealth *health);
    /** @brief 真实确认提交后解除试启动禁止。 @param context 所有者。
     * @param boot_epoch 本次代号。 @param image_token 本次镜像。 @return 身份复核通过返回true。 */
    bool (*confirmation_complete)(void *context, uint64_t boot_epoch, uint32_t image_token);
} BootHandoffOwner;

/**
 * @brief 初始化唯一前台启动交接服务，不擦写记录或触发复位。
 * @param owner 组合层提供的终身端口，复制绑定值，不借用指针。
 * @param context 所有者终身上下文，可为NULL。
 * @param now_ms 单调毫秒时钟。
 * @return 首次有效初始化返回true；缺端口或重复初始化返回false。
 * @note 先由启动最早期执行BootHw_Init，再在APP所有者和通信初始化后调用一次。
 */
bool BootHandoff_Init(const BootHandoffOwner *owner, void *context, uint32_t now_ms);

/**
 * @brief 生成供通信绑定的唯一56值服务。
 * @param service 调用方输出值，失败清零，返回后不保留其地址。
 * @return 已初始化且输出有效返回true；独立APP绑定后请求仍返回UNAVAILABLE。
 * @note 前台组合调用，复制值给通信；不暴露服务内部状态。
 */
bool BootHandoff_GetService(BootEntryService *service);

/**
 * @brief 前台推进停止截止、健康监督与一次试启动确认。
 * @param now_ms 单调毫秒时钟，差值按无符号回绕处理。
 * @note 每100ms核对新监督进展再喂已确认镜像的看门狗；试启动失败不重写确认页。
 *       进入Loader失败或超时持续禁止运动，不自动重新尝试或释放所有者票据。
 */
void BootHandoff_Service(uint32_t now_ms);

#endif
