"""冻结参考代码的配置只读参数/成员路径投影；不改计算、常量、分支或调用次序。"""

import re

_GROUPS = {
    "model": (
        "motor_pole_pairs motor_phase_resistance_ohm motor_d_inductance_h "
        "motor_q_inductance_h motor_flux_wb calibration_current_a id_kp id_ki iq_kp iq_ki"
    ),
    "application": (
        "current_limit_a speed_limit_rad_s speed_acceleration_rad_s2 speed_deceleration_rad_s2 "
        "speed_kp speed_ki position_error_window_rad position_acceleration_rad_s2 "
        "position_deceleration_rad_s2 position_maximum_speed_rad_s "
        "cascade_position_kp cascade_position_kd axis_profile axis_profile_valid"
    ),
    "device": ("phase_a_offset_counts phase_b_offset_counts phase_c_offset_counts"),
}
_FIELDS = {field: group for group, fields in _GROUPS.items() for field in fields.split()}
_PATTERN = re.compile(r"\b(motor_state|m)->(" + "|".join(_FIELDS) + r")\b")
_GROUPED_PATTERN = re.compile(r"\b(?:motor_state|m)->(model|application|device)\.")
_FOC_ENTRIES = r"FOC_Current|FOC_CurrentWithReference|FOC_Vq_Mode"
_SIGNATURE = re.compile(
    r"\b(void|bool|float)\s+(" + _FOC_ENTRIES + r")\((?!\s*const MotorConfiguration)\s*"
)
_CALL = re.compile(
    r"\b(" + _FOC_ENTRIES + r")\((?!\s*(?:const MotorConfiguration|configuration\s*,))\s*"
)


def project_reference_motor_config(source: str) -> str:
    """为旧FOC入口增加只读配置借用；原参考数据流与控制数学保持不变。"""
    source = source.replace('#include "utils.h"', '#include "control_math.h"')
    source = source.replace('#include "foc_errhandle.h"', '#include "motor_fault.h"')
    source = _PATTERN.sub(lambda match: f"configuration->{_FIELDS[match[2]]}.{match[2]}", source)
    source = _GROUPED_PATTERN.sub(r"configuration->\1.", source)
    source = _SIGNATURE.sub(r"\1 \2(const MotorConfiguration *configuration, ", source)
    return _CALL.sub(r"\1(configuration, ", source)
