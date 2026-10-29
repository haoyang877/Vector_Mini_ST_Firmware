// clang-format off
#ifndef FIRMWARE_BOARD_CONFIG_H
#define FIRMWARE_BOARD_CONFIG_H

/* 板卡硬件资源。 */
#include "config.h"

/* AT32 正式工程保持硬件关相。 */
#if defined(FIRMWARE_TARGET_AT32_OFF)
#define BSP_PWM_HARD_LOCK              1
#define APP_TARGET_R40_F456            1
#define BSP_APP_LIFECYCLE              1
#define APP_ENFORCE_POWER_CONTRACT     1
#define APP_FAST_MAX_CYCLES            (R40_CORE_HZ / FOC_FREQ)
#define BSP_ENCODER_FAST_SPI           1
#define BSP_ENCODER_WORD_TRANSFER      1
#define BSP_ENCODER_DMA_TRANSFER       1
#define BSP_ENCODER_TIMEOUT_US         40U
#else
#define BSP_PWM_HARD_LOCK              0
#endif

#if BSP_BOARD == BSP_BOARD_AT32F456

/* 本板配置只能装配到 AT32F456 的 BSP 实现。 */
#define BSP_CONFIG_BOARD_FAMILY  BSP_BOARD_AT32F456
/* 目标默认使用与本采样链配套的软件保护策略。 */
#define BSP_CURRENT_SENSE_POLICY_AT32F456  1

/* 已接线并有实现的能力；原码诊断不代表校准合格或可用于闭环。 */
#define BSP_SUPPORTED_LED                1
#define BSP_SUPPORTED_UART               0
#define BSP_SUPPORTED_PHASE_CURRENT      1
#define BSP_SUPPORTED_BUS_VOLTAGE        1
#define BSP_SUPPORTED_BUS_CURRENT        1
#define BSP_SUPPORTED_ROTOR_ENCODER      1
#define BSP_SUPPORTED_PWM                1
#define BSP_SUPPORTED_CAN                1
#define BSP_SUPPORTED_MCU_TEMPERATURE    1
#define BSP_SUPPORTED_MOTOR_TEMPERATURE  1
/* 提供两路线性霍尔原码观测；不表示已有输出轴角度解算或控制。 */
#define BSP_SUPPORTED_OUTPUT_SINCOS      1

/* 本板逻辑采样顺序及原子输出要求。 */
#define BSP_SAMPLE_PHASE_A_RANK  0U
#define BSP_SAMPLE_PHASE_B_RANK  1U
#define BSP_SAMPLE_PHASE_C_RANK  2U
#define BSP_SAMPLE_VBUS_RANK     3U

#define BSP_CAN_RESERVE_UNTRACKED_SLOT    1
#define BSP_CAN_RECOVERY_CRITICAL         1
#define BSP_TEMPERATURE_SOFTWARE_TRIGGER  0
#define BSP_AUX_MAX_AGE_MS                2U
#define BSP_TEMPERATURE_MAX_AGE_MS        2U
#define BSP_ENCODER_SPI_MAX_HZ            16000000U
#define BSP_PWM_DEADTIME_NS               200U

/* R40模拟链路的实板输入。保护限值和完整APP目标选择留待独立定版。 */
#define R40_ADC_REFERENCE_V       3.3f
#define R40_ADC_RESOLUTION_BITS   12U
#define R40_ADC_MAX_COUNTS        ((1UL << R40_ADC_RESOLUTION_BITS) - 1UL)
#define R40_ADC_ACCUMULATIONS     4U
#define R40_PHASE_SHUNT_MILLIOHM  4U
#define R40_PHASE_SHUNT_OHM       ((float)R40_PHASE_SHUNT_MILLIOHM / 1000.0f)
#define R40_CURRENT_GAIN          20.0f
#define R40_BUS_SHUNT_OHM         0.002f
#define R40_BUS_CURRENT_GAIN      20.0f
#define R40_VBUS_TOP_KOHM         100.0f
#define R40_VBUS_BOTTOM_KOHM      3.3f

/* CN5: 3V3--NTC--TEMP_MOT_IN--R92(1k)--GND；R91(22Ω)只串在ADC滤波支路。
 * 用户确认R25=10kΩ、B=3950K；误差和B值标定温区尚未提供。 */
#define R40_MOTOR_NTC_R25_OHM  10000.0f
#define R40_MOTOR_NTC_BETA_K  3950.0f
#define R40_MOTOR_NTC_PULLDOWN_OHM        1000.0f
#define R40_MOTOR_NTC_RAIL_MARGIN_COUNTS  2U
/* 诊断换算有效窗口，不是传感器额定范围或电机过温跳闸阈值。 */
#define R40_MOTOR_NTC_MIN_C  (-40.0f)
#define R40_MOTOR_NTC_MAX_C  150.0f

/* 必须来自此板实测的温度校准，不能把数据手册典型值当作逐颗出厂校准。
 * 尚未提供校准数据时，温度端口仍报告新原始样本，但conversion_ok保持false。
 * 斜率为实测有符号dVts/dT=(V2-V1)/(T2-T1)，不是手册公式的Avg_Slope。
 * 可由目标编译配置提供实测值；不修改公共BSP或参数记录ABI。 */

#define R40_TEMPERATURE_CALIBRATED  0
#define R40_TEMPERATURE_CAL_C  25.0f
#define R40_TEMPERATURE_CAL_MV  0.0f
#define R40_TEMPERATURE_SLOPE_MV_PER_C  0.0f
#define R40_INTERNAL_REFERENCE_MV  0.0f


/* 完整 APP 关相接入使用实板采样尺度，保护阈值由应用配置提供。 */
#define CURRENT_SENSE_PROFILE_ADC_REFERENCE_V            R40_ADC_REFERENCE_V
#define CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS        R40_ADC_RESOLUTION_BITS
#define CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN             R40_CURRENT_GAIN
#define CURRENT_SENSE_SHUNT_MILLIOHM                     R40_PHASE_SHUNT_MILLIOHM
#define MOTOR_SENSING_VBUS_R1_KOHM                       R40_VBUS_TOP_KOHM
#define MOTOR_SENSING_VBUS_R2_KOHM                       R40_VBUS_BOTTOM_KOHM
#define MOTOR_SENSING_ADC_SAMPLE_COUNT                   R40_ADC_ACCUMULATIONS
#define PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM  0.0f
#define CURRENT_SENSE_PROFILE_RELIABLE_LIMIT_A           20.0f

/* 独立母线电流为12位单次域，与三相四次和域分开。 */
#define BUS_CURRENT_ADC_MAX_COUNTS   R40_ADC_MAX_COUNTS
#define BUS_CURRENT_ADC_REFERENCE_V  R40_ADC_REFERENCE_V
#define BUS_CURRENT_GAIN             R40_BUS_CURRENT_GAIN
#define BUS_CURRENT_SHUNT_OHM        R40_BUS_SHUNT_OHM

/* 三相逻辑通道与编码器资源绑定。 */
/* 首写前保留256个计数的提交裕量；不改变原有效更新事件。 */
#define BSP_PWM_COMMIT_GUARD_COUNTS  256U
#define BSP_PWM_PHASE_A_CHANNEL      2U
#define BSP_PWM_PHASE_B_CHANNEL      3U
#define BSP_PWM_PHASE_C_CHANNEL      4U
#define BSP_ENCODER_DEVICE_TYPE      3U

/* SPI接线由本板配置提供，GPIO名称在对应SDK编译单元中解析。 */
#define BSP_ENCODER_GPIO_PORT   GPIOB
#define BSP_ENCODER_GPIO_CLOCK  CRM_GPIOB_PERIPH_CLOCK
#define BSP_ENCODER_CS_PIN      12U
#define BSP_ENCODER_SCK_PIN     13U
#define BSP_ENCODER_MISO_PIN    14U
#define BSP_ENCODER_MOSI_PIN    15U
#define BSP_ENCODER_GPIO_AF     5U

/* 实板时钟与CAN位时序，数值沿用已验证目标。 */
#define R40_HEXT_HZ          8000000U
#define R40_PLL_MULTIPLIER   90U
#define R40_PLL_PRE_DIVIDER  1U
/* MCU 初始化固定使用 PLL /4；时钟由晶振和实际 PLL 输入派生。 */
#define R40_CORE_CLOCK_DERIVED_HZ                                                                  \
    ((R40_HEXT_HZ * 1ULL * R40_PLL_MULTIPLIER) / (R40_PLL_PRE_DIVIDER * 4ULL))
#define R40_CORE_HZ   ((uint32_t)R40_CORE_CLOCK_DERIVED_HZ)
#define R40_TIMER_HZ  R40_CORE_HZ
#define R40_CAN_HZ    R40_CORE_HZ
/* CAN 固定 1 Mbps / 5 Mbps、75% 采样点；分频为 1。 */
#define R40_CAN_NOMINAL_SEG2  ((uint32_t)(R40_CORE_CLOCK_DERIVED_HZ / 4000000ULL))
#define R40_CAN_NOMINAL_SEG1  (R40_CAN_NOMINAL_SEG2 * 3U)
#define R40_CAN_DATA_SEG2     ((uint32_t)(R40_CORE_CLOCK_DERIVED_HZ / 20000000ULL))
#define R40_CAN_DATA_SEG1     (R40_CAN_DATA_SEG2 * 3U)

#define BSP_LED_SHARED_OUTPUTS  1
#define BSP_LED_RED_PIN         (1U << 13U)
#define BSP_LED_BLUE_PIN        (1U << 14U)

/* AT32F456CCU7 256KiB主Flash，末16KiB仅供参数。与旧板物理地址隔离。
 * 当前RAW记录2288字节，擦除只覆盖前两个2KiB扇区，后续扩展区保持原样。
 * 完整APP身份/Loader布局仍由后续目标打包定义，不在这里伪造产品描述符。 */
#define R40_FLASH_BASE          0x08000000U
#define R40_FLASH_BYTES         0x00040000U
#define R40_FLASH_SECTOR_BYTES  0x800U
#define R40_PARAM_BASE          0x0803C000U
#define R40_PARAM_BYTES         0x4000U
#define R40_PARAM_RAW_BYTES     2288U


#elif BSP_BOARD == BSP_BOARD_STM32G4
/* 本板配置只能装配到 STM32G4 的 BSP 实现。 */
#define BSP_CONFIG_BOARD_FAMILY  BSP_BOARD_STM32G4
/* 目标默认使用与本采样链配套的软件保护策略。 */
#define BSP_CURRENT_SENSE_POLICY_STM32G4  1

/* 只声明已有驱动的能力；预留引脚和FOC估算不作为物理传感器能力。 */
#define BSP_SUPPORTED_LED                1
#define BSP_SUPPORTED_UART               0
#define BSP_SUPPORTED_PHASE_CURRENT      1
#define BSP_SUPPORTED_BUS_VOLTAGE        1
#define BSP_SUPPORTED_BUS_CURRENT        0
#define BSP_SUPPORTED_ROTOR_ENCODER      1
#define BSP_SUPPORTED_PWM                1
#define BSP_SUPPORTED_CAN                1
#define BSP_SUPPORTED_MCU_TEMPERATURE    1
#define BSP_SUPPORTED_MOTOR_TEMPERATURE  0
#define BSP_SUPPORTED_OUTPUT_SINCOS      0

/* 本板逻辑采样顺序及原子输出要求。 */
#define BSP_SAMPLE_PHASE_A_RANK  0U
#define BSP_SAMPLE_PHASE_B_RANK  1U
#define BSP_SAMPLE_PHASE_C_RANK  2U
#define BSP_SAMPLE_VBUS_RANK     3U

#define BSP_CAN_RESERVE_UNTRACKED_SLOT    0
#define BSP_CAN_RECOVERY_CRITICAL         0
#define BSP_TEMPERATURE_SOFTWARE_TRIGGER  1
#define BSP_AUX_MAX_AGE_MS                2U
#define BSP_TEMPERATURE_MAX_AGE_MS        2U

/* STM32 板卡当前使用 6 mΩ 分流电阻及对应路径补偿。
 * 这里只保存目标数据，不含外设、运行状态或在线参数；不由ADC满量程推算安全限值。 */

/* ADC额定输入：电流/VBUS换算不切换到温度通道测得的VDDA。 */
#define CURRENT_SENSE_PROFILE_ADC_REFERENCE_V      3.3f
#define CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS  12U
#define CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN       10.0f

/* 母线分压电阻（kΩ）及ADC每个rank的实际累加次数。 */
#define MOTOR_SENSING_VBUS_R1_KOHM      10.0f
#define MOTOR_SENSING_VBUS_R2_KOHM      1.0f
#define MOTOR_SENSING_ADC_SAMPLE_COUNT  4U

#define CURRENT_SENSE_SHUNT_MILLIOHM  6U
#define PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM  0.008f
#define CURRENT_SENSE_PROFILE_RELIABLE_LIMIT_A           20.0f


/* 三相逻辑通道与编码器资源绑定。 */
/* 首写前保留256个计数的提交裕量；不改变原有效更新事件。 */
#define BSP_PWM_COMMIT_GUARD_COUNTS  256U
#define BSP_PWM_PHASE_A_CHANNEL      1U
#define BSP_PWM_PHASE_B_CHANNEL      2U
#define BSP_PWM_PHASE_C_CHANNEL      3U
#define BSP_ENCODER_DEVICE_TYPE      1U

/* SPI接线由本板配置提供，GPIO名称在对应SDK编译单元中解析。 */
#define BSP_ENCODER_GPIO_PORT  GPIOB
#define BSP_ENCODER_CS_PIN     12U
#define BSP_ENCODER_SCK_PIN    13U
#define BSP_ENCODER_MISO_PIN   14U
#define BSP_ENCODER_MOSI_PIN   15U
#define BSP_ENCODER_GPIO_AF    5U

#define BSP_LED_SHARED_OUTPUTS  0

#else
#error "Unknown BSP board"
#endif /* BSP_BOARD */
/* 仅提供板卡资源声明；不包含应用参数、标定或运行状态。 */

/* 功能直接由板卡资源决定，不再逐项提供使能开关。 */
#define BSP_ENABLED_LED            BSP_SUPPORTED_LED
#define BSP_ENABLED_UART           BSP_SUPPORTED_UART
#define BSP_ENABLED_PHASE_CURRENT  BSP_SUPPORTED_PHASE_CURRENT
#define BSP_ENABLED_BUS_VOLTAGE    BSP_SUPPORTED_BUS_VOLTAGE
#define BSP_ENABLED_BUS_CURRENT    BSP_SUPPORTED_BUS_CURRENT
#define BSP_ENABLED_ROTOR_ENCODER  BSP_SUPPORTED_ROTOR_ENCODER
#define BSP_ENABLED_PWM            BSP_SUPPORTED_PWM
#define BSP_ENABLED_CAN            BSP_SUPPORTED_CAN
#define BSP_ENABLED_MCU_TEMPERATURE        BSP_SUPPORTED_MCU_TEMPERATURE
#define BSP_ENABLED_MOTOR_TEMPERATURE      BSP_SUPPORTED_MOTOR_TEMPERATURE
#define BSP_ENABLED_OUTPUT_SINCOS          BSP_SUPPORTED_OUTPUT_SINCOS
#define BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE  BSP_ENABLED_MCU_TEMPERATURE
#define BSP_ENABLED_HARDWARE_VERSION       0
#define BSP_ENABLED_INTERNAL_ANALOG_SAMPLING                                                       \
    (BSP_ENABLED_MCU_TEMPERATURE || BSP_ENABLED_ANALOG_SUPPLY_VOLTAGE)
#define BSP_ENABLED_AUX_SAMPLING  (BSP_ENABLED_MOTOR_TEMPERATURE || BSP_ENABLED_OUTPUT_SINCOS)

/* 初始中点不是实测零点；三相仍分别在启动采样后取得各自偏置。 */
#define PARAM_HW_CURRENT_OFFSET_A_COUNTS  (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))
#define PARAM_HW_CURRENT_OFFSET_B_COUNTS  (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))
#define PARAM_HW_CURRENT_OFFSET_C_COUNTS  (1UL << (CURRENT_SENSE_PROFILE_ADC_RESOLUTION_BITS - 1U))

#endif
// clang-format on
