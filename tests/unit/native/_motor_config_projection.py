"""冻结参考代码的参数成员路径投影；不替换计算、常量、分支或调用次序。"""

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
    "device": (
        "phase_a_offset_counts phase_b_offset_counts phase_c_offset_counts "
        "friction_coulomb_pos_a friction_coulomb_neg_a friction_viscous_pos_a_per_rad_s "
        "friction_viscous_neg_a_per_rad_s friction_model_valid"
    ),
}
_FIELDS = {field: group for group, fields in _GROUPS.items() for field in fields.split()}
_PATTERN = re.compile(r"\b(motor_state|m)->(" + "|".join(_FIELDS) + r")\b")


def project_reference_motor_config(source: str) -> str:
    """将旧 MotorControlState 参数成员投影到新子对象，保持冻结控制体不变。"""
    return _PATTERN.sub(lambda match: f"{match[1]}->{_FIELDS[match[2]]}.{match[2]}", source)
