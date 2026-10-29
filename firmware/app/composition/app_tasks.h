#ifndef APP_TASKS_H
#define APP_TASKS_H
/**
 * @brief 初始化应用事件计数、测试策略和诊断对象，不访问硬件寄存器。
 * @note App_Init前段调用一次，BSP尚未开放控制事件。
 */
void AppTasks_Init(void);
/**
 * @brief 在采样和通信就绪后启动应用健康监督。
 * @note App_Start前台调用一次，看门狗硬件动作经BSP执行。
 */
void AppTasks_Start(void);
/**
 * @brief 推进应用诊断、功率窗口和健康喂狗判定。
 * @note App_Process末尾有界调用；不解析协议、不执行Flash或启动电机。
 */
void AppTasks_Process(void);
#endif
