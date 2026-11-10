#ifndef APP_H
#define APP_H

#include <stdint.h>
#include "motor_axis.h"
#include "app_protocol.h"
#include "app_motor_configuration.h"
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
#include "app_loader.h"
#endif
#include "device_identity.h"

/** @brief 由板级静态分配并零初始化，绑定后地址稳定；每个实例独占这些对象。 */
typedef struct
{
    MotorAxis axis;
    AppProtocol protocol;
    AppMotorConfiguration motor_configuration;
    ProtocolEndpoint endpoint;
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    AppLoader loader;
#endif
    DeviceIdentity identity;
    bool communication_ready;
    bool initialized;
} AppContext;

/**
 * @brief 装载配置、初始化电机和通信对象，不开放采样或功率输出。
 * @note main完成芯片外设初始化后调用一次；随后由main通过Bsp_Start开放采样。
 *       初始三相保持关闭，通信初始化失败沿原路径关相；不代替主状态机的启动准入。
 * @param app 板级零初始化的应用实例。
 */
void App_Init(AppContext *app);

/**
 * @brief 在BSP采样启动后开放通信。
 * @note main前台一次调用；失败保持关相，不恢复运动。
 * @param app 板级零初始化的应用实例。
 */
void App_Start(AppContext *app);

/**
 * @brief 按顺序推进后台事务、通信及 Loader 交接。
 * @note 仅在main前台循环调用；不能从中断或并发上下文调用。
 *       通信和交接分别读取当时毫秒时钟，不能合并为提前捕获的时间快照。
 * @param app 板级零初始化的应用实例。
 */
void App_Process(AppContext *app);

/**
 * @brief 初始化位置环反馈 RTT 上行通道。
 * @note 由启用RTT的板级入口在启动采样中断前调用一次，不启相。
 */
void RTT_Init(void);

/**
 * @brief 通道 1 发送 Hall 单圈角（mrad）及 sin/cos 原码。
 * @note 仅在初始化后的 2 kHz 监督回调完成后调用；单写者，缓冲满则丢整帧，不阻塞。
 *       换算按int16饱和并截断取整，启动零偏未就绪的NaN记为0。
 */
void RTT_Sampling(void);

#endif
