#ifndef APP_H
#define APP_H

#include <stdint.h>

/**
 * @brief 已完成的轴快环次数，供应用看门狗观察进展。
 * @note 仅ADC快环完成后递增；App_Init在开放采样前清零，前台只读，无符号回绕。
 */
extern volatile uint32_t g_app_fast_samples;

/**
 * @brief 装载配置、初始化电机和通信对象，不开放采样或功率输出。
 * @note main完成芯片外设初始化后调用一次；随后由main通过Bsp_Start开放采样。
 *       初始三相保持关闭，通信初始化失败沿原路径关相；不代替主状态机的启动准入。
 */
void App_Init(void);

/**
 * @brief 在BSP采样启动后开放通信并开始应用健康监督。
 * @note main前台一次调用；失败保持关相，不恢复运动。
 */
void App_Start(void);

/**
 * @brief 按顺序推进后台事务、状态刷新、通信及Loader交接。
 * @note 仅在main前台循环调用；不能从中断或并发上下文调用。
 *       通信和交接分别读取当时毫秒时钟，不能合并为提前捕获的时间快照。
 */
void App_Process(void);

/**
 * @brief 执行一次2 kHz电机监督及主状态推进。
 * @note 仅由平台2kHz监督中断调用；不阻塞。
 */
void App_SupervisorTick(void);

/**
 * @brief 初始化六路浮点RTT上行缓冲区。
 * @note 由启用RTT的板级入口在启动采样中断前调用一次，不启相。
 */
void RTT_Init(void);

/**
 * @brief 发送本拍三相电流、电角度、转速反馈和q轴电流反馈。
 * @note 仅在初始化后的ADC快环采样与控制完成后调用；单写者，缓冲满则丢整帧，不阻塞。
 */
void RTT_Sampling(void);

#endif
