#include "foc_task.h"

#include "angle_feedback.h"
#include "foc_mode_dispatch.h"
#include "foc_run.h"
#include "foc_run_state.h"
#include "foc_sensing.h"
#include "foc_sensorless.h"
#include "motor_state.h"
#include "rtt_telemetry.h"

/* 三层执行入口：20 kHz 快环只做采样、电角度、电流环与运行状态；
 * 2 kHz 监督 tick 承担慢估计、外环控制与温度；后台主循环负责异步会话。 */

void FOC2kHzSupervisor(void)
{
    /* 1. 编码器慢估计：多圈/机械角/速度是慢状态唯一写者。 */
    Encoder_UpdateSlowEstimate(&MotorControl, &OnBoard_Encoder);

    /* 2. 外环控制：位置级联/轨迹与速度 PI 直接执行并发布 iqRef；
     *    本函数是外环输出唯一写者，快环只读取。 */
    MotorOuterLoop_SlowTick(&MotorControl, &PI_Speed, &OnBoard_Encoder);

    /* 3. 主运行状态机：消费快环 worker 结果，完成故障指示、维护会话、
     *    功率级启停迁移与影子提交；故障的硬件关断由快环紧急快车道先行执行。 */
    FocRunState_Tick();

    /* 4. MCU 温度与 VREFINT 换算属于 2 kHz 监督任务。 */
    Temperature_Update(&FOC);
}

void FOC20kHzIRQHandler(void)
{
    /* 本拍模式 worker 的周期结果，仅由运行状态机消费。 */
    MotorWorkOutcome_TypeDef work_outcome;
    /* 编码器帧采样是否已在本拍成功发起；false 时稍后走同步兜底读取。 */
    bool encoder_sample_started;

    /* 1. 发起编码器帧采样：只启动传感器总线事务，不在本拍等待结果。
     *    返回 false 表示总线未就绪，由第 4 步同步兜底完成本帧。 */
    encoder_sample_started = Encoder_BeginSample();

    /* 2. 母线电压采样：换算 Vbus、更新低通滤波值，并推进过压/欠压确认计数；
     *    禁用模式清零计数。保护判定基于本拍电压，必须先于控制输出执行。 */
    Vbus_Update(&FOC, &MotorControl);

    /* 3. 相电流换算：ADC 原始值减零点偏置并折算为 Ia/Ib/Ic（A），
     *    同时校验偏置有效性，并按连续 5 拍确认过流。 */
    Current_Cal(&FOC, &MotorControl);

    /* 4. 完成本拍角度采样：取回 SPI 帧，做方向/LUT 校正并更新电角度；
     *    多圈/机械角/速度由 2 kHz 慢估计维护。 */
    Encoder_CompleteSample(&MotorControl, &OnBoard_Encoder, encoder_sample_started);

    /* 5. 观测器选择：位置模式（模式 3）由编码器与外环提供角度，跳过观测器；
     *    其余模式在模式分发前更新，首次切进电流/无感模式的这一拍即有估计可用。 */
    if (MotorControl.ModeNow != Position_Mode)
    {
        Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);
    }

    /* 6. 快速环故障检查：编码器离线或轴配置失效时立即置故障并停机；
     *    不清除既有故障，且先于任何控制输出执行。 */
    FocRunState_CheckFastFaults();

    /* 7. 故障紧急关断快车道：故障锁存时立即切断功率级（无硬件 BKIN，软件是
     *    唯一关断路径）；状态机在下一 2 kHz 慢拍对齐记录并清理。 */
    FocRunState_FastFaultStop();

    /* 8. 模式分发：按 ModeNow 执行对应 worker（禁用态零矢量预载、控制、
     *    标定与参数命令），回传周期结果；本函数不写 ModeNow 或功率级。 */
    work_outcome = FocMode_Dispatch();

    /* 9. worker 结果入箱：由 2 kHz 主状态机在下一慢拍消费。 */
    FocRunState_PostOutcome(work_outcome);

    /* 10. 遥测：采样一帧 4 通道 RTT（机械位置/机械转速/Iq 指令/Iq 反馈），
     *     非阻塞、有界，不修改控制状态与运行模式。 */
    RTT_Sampling();
}
