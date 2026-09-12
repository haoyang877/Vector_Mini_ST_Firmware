"""Field table and parser for the flash parameter record (schema v10, 2240 bytes).

Layout source: ``Foc/foc_param.h`` (``InterfaceParam_TypeDef``) and
``software/config/motor_axis_profile.h``. Offsets are fixed by the C struct;
``RECORD_SIZE`` and ``LUT_*`` are validated against the firmware INFO reply.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct

RECORD_SIZE = 2240
Q15_CPR = 65536.0
LUT_OFFSET = 0x2C
LUT_COUNT = 1024
AXIS_PROFILE_OFFSET = 0x8A0
AXIS_PROFILE_SIZE = 32
AXIS1_MAGIC = 0x31535841
AXIS2_MAGIC = 0x32535841
RECORD_MAGIC = 0x454E4332


@dataclass(frozen=True)
class FlashField:
    """One scalar field of the flash record."""

    name: str
    offset: int
    fmt: str  # struct format char: f / H / B / I
    unit: str
    group: str


FIELDS: tuple[FlashField, ...] = (
    FlashField("node_id", 0x000, "f", "-", "基础"),
    FlashField("currentoffset_a", 0x004, "f", "A", "电流标定"),
    FlashField("currentoffset_b", 0x008, "f", "A", "电流标定"),
    FlashField("currentoffset_c", 0x00C, "f", "A", "电流标定"),
    FlashField("motor_pole_pairs", 0x010, "f", "对", "电机参数"),
    FlashField("motor_phase_resistance", 0x014, "f", "Ω", "电机参数"),
    FlashField("motor_d_inductance", 0x018, "f", "H", "电机参数"),
    FlashField("motor_q_inductance", 0x01C, "f", "H", "电机参数"),
    FlashField("motor_flux", 0x020, "f", "Wb", "电机参数"),
    FlashField("encoder_electrical_zero_q15", 0x024, "H", "q15", "编码器"),
    FlashField("encoder_mechanical_zero_q15", 0x026, "H", "q15", "编码器"),
    FlashField("encoder_calib_flag", 0x028, "B", "-", "编码器"),
    FlashField("encoder_reverse", 0x029, "B", "-", "编码器"),
    FlashField("id_kp", 0x82C, "f", "-", "电流环"),
    FlashField("id_ki", 0x830, "f", "-", "电流环"),
    FlashField("iq_kp", 0x834, "f", "-", "电流环"),
    FlashField("iq_ki", 0x838, "f", "-", "电流环"),
    FlashField("speedAcc", 0x83C, "f", "rad/s²", "速度环"),
    FlashField("speedDec", 0x840, "f", "rad/s²", "速度环"),
    FlashField("speed_kp", 0x844, "f", "-", "速度环"),
    FlashField("speed_ki", 0x848, "f", "-", "速度环"),
    FlashField("posAcc", 0x84C, "f", "rad/s²", "位置环"),
    FlashField("posDec", 0x850, "f", "rad/s²", "位置环"),
    FlashField("pos_maxspeed", 0x854, "f", "rad/s", "位置环"),
    FlashField("pos_kp", 0x858, "f", "A/rad", "位置环"),
    FlashField("pos_kd", 0x85C, "f", "A/(rad/s)", "位置环"),
    FlashField("calib_current", 0x860, "f", "A", "电流标定"),
    FlashField("current_limit", 0x864, "f", "A", "保护"),
    FlashField("speed_limit", 0x868, "f", "rad/s", "保护"),
    FlashField("can_hb", 0x86C, "f", "ms", "CAN"),
    FlashField("schema_version", 0x870, "I", "-", "基础"),
    FlashField("magic_word", 0x874, "I", "-", "基础"),
    FlashField("pos_ki", 0x878, "f", "A/(rad·s)", "位置环"),
    FlashField("current_sense_shunt_milliohm", 0x87C, "I", "mΩ", "电流标定"),
    FlashField("pos_integral_limit", 0x880, "f", "A", "位置环"),
    FlashField("cascade_pos_kp", 0x884, "f", "1/s", "位置环"),
    FlashField("cascade_pos_kd", 0x888, "f", "-", "位置环"),
    FlashField("friction_coulomb_pos_a", 0x88C, "f", "A", "摩擦"),
    FlashField("friction_coulomb_neg_a", 0x890, "f", "A", "摩擦"),
    FlashField("friction_viscous_pos_a_per_rad_s", 0x894, "f", "A/(rad/s)", "摩擦"),
    FlashField("friction_viscous_neg_a_per_rad_s", 0x898, "f", "A/(rad/s)", "摩擦"),
    FlashField("friction_model_valid", 0x89C, "I", "-", "摩擦"),
)


class FlashRecordParseError(ValueError):
    """The record blob is too short or structurally invalid."""


def parse_record(blob: bytes) -> dict[str, object]:
    """Parse the raw record into named fields plus LUT and axis profile."""
    if len(blob) < RECORD_SIZE:
        raise FlashRecordParseError(f"record too short: {len(blob)} < {RECORD_SIZE}")
    result: dict[str, object] = {}
    for field in FIELDS:
        value = struct.unpack_from("<" + field.fmt, blob, field.offset)[0]
        result[field.name] = value if field.fmt in ("H", "B", "I") else round(value, 9)
    result["encoder_linearization_lut_q15"] = list(struct.unpack_from("<1024h", blob, LUT_OFFSET))
    result["axis_profile"] = _parse_axis_profile(blob)
    return result


def _parse_axis_profile(blob: bytes) -> dict[str, object]:
    raw = blob[AXIS_PROFILE_OFFSET : AXIS_PROFILE_OFFSET + AXIS_PROFILE_SIZE]
    magic, version = struct.unpack_from("<II", raw, 0)
    profile: dict[str, object] = {"magic": magic, "version": version}
    if magic == AXIS2_MAGIC:
        joint_type, config_revision = struct.unpack_from("<II", raw, 8)
        profile["joint_type"] = joint_type
        profile["config_revision"] = config_revision
    elif magic == AXIS1_MAGIC:
        name = raw[8:16].split(b"\x00", 1)[0]
        profile["legacy_name"] = name.decode("ascii", "replace")
    else:
        return {"magic": magic, "version": version, "configured": False}
    minimum, maximum, max_speed = struct.unpack_from("<fff", raw, 16)
    crc32 = struct.unpack_from("<I", raw, 28)[0]
    profile.update(
        {
            "configured": True,
            "minimum_position_rad": minimum,
            "maximum_position_rad": maximum,
            "maximum_speed_rad_s": max_speed,
            "crc32": crc32,
        }
    )
    return profile


def lut_angle_error_degrees(blob: bytes) -> list[float]:
    """Per-LUT-point angle error in degrees (firmware USB-export formula)."""
    lut = struct.unpack_from("<1024h", blob, LUT_OFFSET)
    return [value * 360.0 / Q15_CPR for value in lut]


def lut_raw_angle_degrees() -> list[float]:
    """Angle in degrees represented by each LUT point (index * 64 in q15)."""
    return [index * 64.0 * 360.0 / Q15_CPR for index in range(LUT_COUNT)]


def lut_statistics(blob: bytes) -> dict[str, float | int]:
    """Summary of the angle-error curve: max/rms/peak-to-peak in degrees."""
    errors = lut_angle_error_degrees(blob)
    max_value = max(errors)
    min_value = min(errors)
    rms = (sum(value * value for value in errors) / len(errors)) ** 0.5
    max_index = errors.index(max_value)
    return {
        "points": len(errors),
        "max_deg": max_value,
        "max_index": max_index,
        "max_angle_deg": max_index * 64.0 * 360.0 / Q15_CPR,
        "min_deg": min_value,
        "peak_to_peak_deg": max_value - min_value,
        "rms_deg": rms,
    }
