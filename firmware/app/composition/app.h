#ifndef APP_H
#define APP_H

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
 * @note 仅在main前台循环调用；Flash保存留在前台，不能从中断或并发上下文调用。
 *       通信和交接分别读取当时毫秒时钟，不能合并为提前捕获的时间快照。
 */
void App_Process(void);

/**
 * @brief 执行一次20 kHz电机采样、快速保护及控制或标定步骤。
 * @note 仅由平台完整ADC采样回调调用；保持既有立即关断路径，不阻塞或执行Flash操作。
 */
void App_FastTick(void);

/**
 * @brief 执行一次2 kHz电机监督、主状态推进及指示器分频。
 * @note 仅由平台2kHz监督中断调用；LED/RGB在电机监督之后按原分频执行，不替代快环故障关断。
 */
void App_SupervisorTick(void);

#endif
