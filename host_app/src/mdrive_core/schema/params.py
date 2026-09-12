"""Authoritative revision-2 CAN parameter catalog derived from interface_can."""
# noqa: SIZE_OK - this module is an indivisible firmware protocol data table.

from dataclasses import dataclass
import math
from typing import Literal

Access = Literal["rw", "ro", "cmd"]
Encoding = Literal["f32", "ma16", "mrad32", "crad32"]


@dataclass(frozen=True, slots=True)
class ParamDef:
    """One addressable firmware parameter or command."""

    id: int
    name: str
    group: str
    access: Access
    encoding: Encoding
    unit: str
    minimum: float | None
    maximum: float | None
    get_id: int | None
    note: str = ""


GROUPS = ["模式", "电流", "速度", "位置", "位置阻抗", "摩擦", "电机模型", "CAN", "遥测"]


def _definition(
    identifier: int,
    name: str,
    group: str,
    access: Access,
    encoding: Encoding = "f32",
    unit: str = "",
    minimum: float | None = None,
    maximum: float | None = None,
    get_id: int | None = None,
    note: str = "",
) -> ParamDef:
    return ParamDef(identifier, name, group, access, encoding, unit, minimum, maximum, get_id, note)


PARAMS: dict[int, ParamDef] = {}


def _add_pair(
    set_id: int,
    set_name: str,
    get_name: str,
    group: str,
    encoding: Encoding = "f32",
    unit: str = "",
    minimum: float | None = None,
    maximum: float | None = None,
    note: str = "",
) -> None:
    get_id = set_id + 1
    PARAMS[set_id] = _definition(set_id, set_name, group, "rw", encoding, unit, minimum, maximum, get_id, note)
    PARAMS[get_id] = _definition(get_id, get_name, group, "ro", encoding, unit, minimum, maximum, None, f"Reply for {set_name}.")


_add_pair(0x00, "SET_MODE", "GET_MODE", "模式", minimum=0, note="Integer mode; firmware requires 0 <= value < MODE_NUM.")
_add_pair(
    0x02,
    "SET_CURRENT",
    "GET_CURRENT_SET",
    "电流",
    "ma16",
    "A",
    note="Absolute command must not exceed the configured current limit.",
)
_add_pair(
    0x04,
    "SET_SPEED",
    "GET_SPEED_SET",
    "速度",
    "crad32",
    "rad/s",
    note="Absolute command must not exceed the configured speed limit.",
)
_add_pair(0x06, "SET_POS", "GET_POS_SET", "位置", "mrad32", "rad")
_add_pair(0x08, "SET_NODE_ID", "GET_NODE_ID", "CAN", minimum=0, maximum=7)
_add_pair(0x0A, "SET_POLEPARIS", "GET_POLEPARIS", "电机模型", minimum=2, maximum=30)
_add_pair(
    0x0C,
    "SET_ENCODER_STATE",
    "GET_ENCODER_STATE",
    "模式",
    minimum=0,
    maximum=1,
    note="Write is accepted only outside active current/speed operation.",
)
_add_pair(
    0x0E,
    "SET_CURRENT_CAL",
    "GET_CURRENT_CAL",
    "电流",
    "ma16",
    "A",
    0,
    10,
    note="Default 6 mOhm profile limit is 10 A; 2 mOhm build profile permits 30 A.",
)
_add_pair(
    0x10,
    "SET_CURRENT_LIMIT",
    "GET_CURRENT_LIMIT",
    "电流",
    "ma16",
    "A",
    0,
    10,
    note="Default 6 mOhm profile limit is 10 A; 2 mOhm build profile permits 30 A.",
)
_add_pair(
    0x12,
    "SET_SPEED_LIMIT",
    "GET_SPEED_LIMIT",
    "速度",
    "crad32",
    "rad/s",
    0,
    math.pi,
    note="Minimum is exclusive; maximum is PARAM_MOTOR_SPEED_LIMIT_RPS * 2*pi.",
)
_add_pair(0x14, "SET_SPEED_ACC", "GET_SPEED_ACC", "速度", "crad32", "rad/s^2", 0, 1000 * 2 * math.pi)
_add_pair(0x16, "SET_SPEED_DEC", "GET_SPEED_DEC", "速度", "crad32", "rad/s^2", 0, 1000 * 2 * math.pi)
_add_pair(0x18, "SET_SPEED_KP", "GET_SPEED_KP", "速度", minimum=0.01, maximum=2.0)
_add_pair(0x1A, "SET_SPEED_KI", "GET_SPEED_KI", "速度", minimum=0, maximum=2.0)
_add_pair(
    0x1C,
    "SET_POS_ACC",
    "GET_POS_ACC",
    "位置",
    "crad32",
    "rad/s^2",
    0,
    200 * 2 * math.pi,
    note="Minimum is exclusive.",
)
_add_pair(
    0x1E,
    "SET_POS_DEC",
    "GET_POS_DEC",
    "位置",
    "crad32",
    "rad/s^2",
    0,
    200 * 2 * math.pi,
    note="Minimum is exclusive.",
)
_add_pair(
    0x20,
    "SET_POS_MAXSPEED",
    "GET_POS_MAXSPEED",
    "位置",
    "crad32",
    "rad/s",
    0,
    math.pi,
    note="Minimum is exclusive and value must also not exceed the configured speed limit.",
)
_add_pair(0x22, "SET_POS_KP", "GET_POS_KP", "位置阻抗", unit="A/rad", minimum=0, maximum=50)
_add_pair(0x24, "SET_POS_KD", "GET_POS_KD", "位置阻抗", unit="A/(rad/s)", minimum=0, maximum=10)
_add_pair(
    0x26,
    "SET_COGGING",
    "GET_COGGING",
    "电机模型",
    minimum=0,
    maximum=1,
    note="Declared in firmware but both handlers are currently disabled.",
)
_add_pair(
    0x28,
    "SET_CAN_BR",
    "GET_CAN_BR",
    "CAN",
    unit="kbps",
    note="Allowed set: 100, 125, 200, 250, 500, 1000, 2000, 2500, 5000. Firmware currently replies on ID 0x2B instead of 0x29.",
)
_add_pair(
    0x2A,
    "SET_CAN_HB",
    "GET_CAN_HB",
    "CAN",
    unit="ms",
    minimum=0,
    maximum=1000,
    note="Allowed values are 0 or 500..1000 ms.",
)


def _add_readonly(identifier: int, name: str, group: str, encoding: Encoding = "f32", unit: str = "", note: str = "") -> None:
    PARAMS[identifier] = _definition(identifier, name, group, "ro", encoding, unit, note=note)


_add_readonly(0x2D, "GET_VBUS", "遥测", unit="V")
_add_readonly(0x2F, "GET_IBUS", "遥测", "ma16", "A")
_add_readonly(0x31, "GET_IA", "遥测", "ma16", "A")
_add_readonly(0x33, "GET_IB", "遥测", "ma16", "A")
_add_readonly(0x35, "GET_IC", "遥测", "ma16", "A")
_add_readonly(0x37, "GET_ID", "遥测", "ma16", "A")
_add_readonly(0x39, "GET_IQ", "遥测", "ma16", "A")
_add_readonly(0x3F, "GET_SPEED2_FILT", "遥测", "crad32", "rad/s")
_add_readonly(0x41, "GET_POS2_FILT", "遥测", "mrad32", "rad")
_add_readonly(0x43, "GET_TEMP", "遥测", unit="degC")
_add_readonly(0x45, "GET_RS", "电机模型", unit="ohm")
_add_readonly(0x47, "GET_LD", "电机模型", unit="H")
_add_readonly(0x49, "GET_LQ", "电机模型", unit="H")
_add_readonly(0x4B, "GET_FLUX", "电机模型", unit="Wb")
_add_readonly(0x4D, "GET_ERROR", "遥测")
_add_pair(0x4E, "SET_ENCODER_REVERSE", "GET_ENCODER_REVERSE", "模式", minimum=0, maximum=1)
_add_pair(0x50, "SET_POS_KI", "GET_POS_KI", "位置阻抗", unit="A/(rad*s)", minimum=0, maximum=10)
_add_pair(
    0x52,
    "SET_POS_INTEGRAL_LIMIT",
    "GET_POS_INTEGRAL_LIMIT",
    "位置阻抗",
    unit="A",
    minimum=0,
    maximum=10,
    note="Maximum follows the selected current-sense profile and can be 30 A.",
)
_add_pair(0x54, "SET_CASCADE_POS_KP", "GET_CASCADE_POS_KP", "位置", unit="1/s", minimum=0, maximum=50)
_add_pair(0x56, "SET_CASCADE_POS_KD", "GET_CASCADE_POS_KD", "位置", minimum=0, maximum=10)

PARAMS[0x58] = _definition(0x58, "APPLY_FRICTION_MODEL", "摩擦", "cmd", minimum=1, maximum=1)
_add_readonly(0x59, "GET_FRICTION_STATE", "摩擦")
_add_readonly(0x5A, "GET_FRICTION_REASON", "摩擦")
_add_readonly(0x5B, "GET_FRICTION_COULOMB_POS", "摩擦", "ma16", "A")
_add_readonly(0x5C, "GET_FRICTION_COULOMB_NEG", "摩擦", "ma16", "A")
_add_readonly(0x5D, "GET_FRICTION_VISCOUS_POS", "摩擦", unit="A/(rad/s)")
_add_readonly(0x5E, "GET_FRICTION_VISCOUS_NEG", "摩擦", unit="A/(rad/s)")
_add_readonly(0x5F, "GET_FRICTION_RMSE_POS", "摩擦", "ma16", "A")
_add_readonly(0x60, "GET_FRICTION_RMSE_NEG", "摩擦", "ma16", "A")
_add_readonly(0x61, "GET_FRICTION_CANDIDATE_VALID", "摩擦")
_add_readonly(0x62, "GET_FRICTION_MODEL_VALID", "摩擦")
PARAMS[0x64] = _definition(
    0x64,
    "SET_STATUS_STREAM",
    "遥测",
    "cmd",
    note="0 stops, 1 resumes, and integral 10..200 selects Hz and starts; reply uses 0x65.",
)
_add_readonly(0x65, "GET_STATUS_STREAM", "遥测", unit="Hz")
PARAMS[0x66] = _definition(0x66, "ENTER_BOOT", "CAN", "cmd", note="Safe-stop and reset into the resident Loader.")
_add_readonly(0x67, "GET_PROTOCOL_REVISION", "CAN", note="Revision 2 selects SI fixed-point parameter and 48-byte status formats.")
PARAMS[0x68] = _definition(0x68, "SAVE_PARAM", "CAN", "cmd", note="Planned; current firmware does not implement this command.")
PARAMS[0x69] = _definition(0x69, "FLASH_PARAM_INFO", "CAN", "cmd", note="Planned read command; current firmware does not implement it.")
PARAMS[0x6A] = _definition(0x6A, "FLASH_PARAM_CHUNK", "CAN", "cmd", note="Planned read command; current firmware does not implement it.")

SET_IDS = frozenset(identifier for identifier, definition in PARAMS.items() if definition.access == "rw")
READABLE_IDS = frozenset(identifier for identifier, definition in PARAMS.items() if definition.access in {"rw", "ro"})


def find_by_get_id(get_id: int) -> ParamDef | None:
    """Return the writable definition paired with ``get_id``."""
    return next((definition for definition in PARAMS.values() if definition.get_id == get_id), None)


def assert_consistent() -> None:
    """Raise AssertionError when schema IDs, groups, or SET/GET pairs drift."""
    if len(PARAMS) != len({definition.id for definition in PARAMS.values()}):
        raise AssertionError("parameter IDs are not unique")
    for identifier, definition in PARAMS.items():
        if definition.id != identifier:
            raise AssertionError(f"schema key {identifier:#x} does not match definition ID")
        if definition.group not in GROUPS:
            raise AssertionError(f"unknown group for {definition.name}: {definition.group}")
        if definition.access == "rw":
            expected_get = identifier + 1
            if definition.get_id != expected_get or expected_get not in PARAMS:
                raise AssertionError(f"invalid SET/GET pair for {definition.name}")
