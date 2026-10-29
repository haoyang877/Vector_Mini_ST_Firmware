#ifndef MOTOR_PROFILE_YQ_5708_H
#define MOTOR_PROFILE_YQ_5708_H

/* YQ-5708星形电机：R/L由线间值折算，Ld/Lq为初始等效估计。
 * 磁链由电机轴转矩常数及Iq幅值反算；完整热参数仍待确认。 */
#define PARAM_MOTOR_MODEL_NAME "YQ-5708"
#define PARAM_MOTOR_CONFIGURATION_NAME "YQ-5708/8:1"
#define PARAM_MOTOR_APP_DEFAULTS_READY 0U
#define PARAM_MOTOR_DATA_PROVISIONAL 0U
#define PARAM_MOTOR_POLE_PAIRS 14
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM (1.1f * 0.5f)
#define PARAM_MOTOR_D_INDUCTANCE_H (0.000285f * 0.5f)
#define PARAM_MOTOR_Q_INDUCTANCE_H (0.000285f * 0.5f)
/* 用户确认Kt=80.1mN·m/A，A为Iq/相电流幅值，参考轴为电机轴。
 * Id=0时Te=1.5*p*磁链*Iq，磁链单位Wb，不按减速比缩放。 */
#define PARAM_MOTOR_TORQUE_CONSTANT_NM_PER_A (80.1f * 0.001f)
#define PARAM_MOTOR_TORQUE_CONSTANT_VALID 1U
#define PARAM_MOTOR_FLUX_WB                                                                        \
    (PARAM_MOTOR_TORQUE_CONSTANT_NM_PER_A / (1.5f * (float)PARAM_MOTOR_POLE_PAIRS))
/* 用户明确SI值为0.0005kg·m²，不采用截图e-4标签再次缩放；未计外部负载惯量。 */
#define PARAM_MOTOR_ROTOR_INERTIA_KG_M2 0.0005f
#define PARAM_MOTOR_ROTOR_INERTIA_VALID 1U
/* 电机规格为向量幅值，不改变板级/台架保护；原始规格为2.5/7.5Arms。 */
#define PARAM_MOTOR_RATED_VECTOR_A (2.5f * 1.414213562f)
#define PARAM_MOTOR_PEAK_VECTOR_A (7.5f * 1.414213562f)
#define PARAM_MOTOR_MAX_SPEED_RPM 2500.0f
#define PARAM_MOTOR_GEAR_RATIO 8.0f
#define PARAM_MOTOR_CALIB_CURRENT_A 1.6f
/* 14对极下30rad/s对应420rad/s电角速度，高于观测器交接下限250rad/s。
 * 标定速度独立于日常速度模式限值，保留既有实机配置。 */
#define PARAM_MOTOR_CALIB_TARGET_ELEC_VEL_RAD_S 420.0f
#define PARAM_MOTOR_CALIB_SPEED_MEC_RAD_S 30.0f

#endif
