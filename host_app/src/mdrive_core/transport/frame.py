"""Transport-neutral CAN frame value."""

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class CanFrame:
    """One received CAN or CAN-FD frame."""

    identifier: int
    data: bytes
    flags: int = 0
    timestamp: float | None = None
