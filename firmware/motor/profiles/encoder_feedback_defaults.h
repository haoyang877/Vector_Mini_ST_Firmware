#ifndef ENCODER_FEEDBACK_DEFAULTS_H
#define ENCODER_FEEDBACK_DEFAULTS_H

/* 机械速度估计的公共默认配置；目标可预先提供经过独立验收的带宽。
 * 默认90Hz保持旧板行为，调试目标的覆盖不改变正式默认值。 */
#ifndef ENCODER_PLL_OMEGA_N_HZ
#define ENCODER_PLL_OMEGA_N_HZ 90.0f
#endif
#define ENCODER_PLL_ZETA 0.707f
#define ENCODER_PLL_OMEGA_MAX_RAD_S 2000.0f
#define ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S 0.05f

#endif
