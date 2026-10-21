#ifndef DEVICE_PROFILE_DEFAULTS_H
#define DEVICE_PROFILE_DEFAULTS_H

/* 首次装配输入：设备通信地址，不用于选择电机型号或应用限值。 */
#define PARAM_DEVICE_CAN_NODE_ID 0x01U

/* 未标定占位：零值不代表已测得零位；实际 LUT/零位由标定任务提交。 */
#define PARAM_DEVICE_ENCODER_ELECTRICAL_ZERO_Q15 0U
#define PARAM_DEVICE_ENCODER_MECHANICAL_ZERO_Q15 0U
#define PARAM_DEVICE_ENCODER_CALIB_FLAG 0U
/* 初始方向配置；改方向须使原标定失效，不能只改此默认值复用旧 Flash。 */
#define PARAM_DEVICE_ENCODER_REVERSE 0U

#endif
