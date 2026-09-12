"""Encode stream commands and decode the firmware's 48-byte motor status."""

from dataclasses import dataclass
import math
import struct

FORMAT = ">HHiiiihhiihh12x"
ID_BASE = 0x7F0
FIELDS = (
    "fault",
    "mode",
    "position_target_rad",
    "position_feedback_rad",
    "speed_target_rad_s",
    "speed_feedback_rad_s",
    "iq_reference_A",
    "iq_feedback_A",
    "position_planned_rad",
    "speed_planned_rad_s",
    "temperature_C",
    "bus_voltage_V",
)
SCALES = (None, None, 1000, 1000, 100, 100, 1000, 1000, 1000, 100, 100, 100)


@dataclass(frozen=True, slots=True)
class StatusSample:
    """Typed wrapper around a decoded status mapping."""

    node: int
    values: dict[str, int | float | None | list[str]]


class StatusValueError(ValueError):
    """A status command or frame violates the validated wire contract."""


def command(node: int, value: float) -> tuple[int, bytes]:
    """Encode 0=stop, 1=resume, or an integral 10..200 Hz stream command."""
    if type(node) is not int or not 0 <= node <= 7:
        raise StatusValueError("node must be 0..7")
    valid_value = value in (0, 1) or 10 <= value <= 200 and value == int(value)
    if not math.isfinite(value) or not valid_value:
        raise StatusValueError("command: 0=stop, 1=resume, integer 10..200=Hz and start")
    return (node << 8) | 0x64, struct.pack(">f", value)


def decode(identifier: int, payload: bytes) -> dict[str, int | float | None | list[str]]:
    """Decode a validated status frame into SI values and sentinel metadata."""
    if not ID_BASE <= identifier <= ID_BASE + 7 or len(payload) != 48:
        raise StatusValueError("status requires standard ID 0x7F0..0x7F7 and 48 bytes")
    values = struct.unpack(FORMAT, payload)
    invalid_fields: list[str] = []
    result: dict[str, int | float | None | list[str]] = {
        "node": identifier - ID_BASE,
        "invalid_fields": invalid_fields,
    }
    for index, (key, value) in enumerate(zip(FIELDS, values, strict=True)):
        if index < 2:
            result[key] = value
            continue
        sentinel = -32768 if index in (6, 7, 10, 11) else -2147483648
        if value == sentinel:
            result[key] = None
            invalid_fields.append(key)
        else:
            scale = SCALES[index]
            if scale is None:
                raise AssertionError("status scale missing")
            result[key] = value / scale
    return result
