"""Shared CAN transport contract."""

from dataclasses import dataclass
from typing import Protocol

from .frame import CanFrame


class TransportError(OSError):
    """A CAN transport operation failed."""


@dataclass(slots=True)  # noqa: MUTABLE_OK - transport implementations maintain these counters.
class TransportStats:
    """Mutable counters owned by a transport instance."""

    tx_count: int = 0
    rx_count: int = 0
    error_frames: int = 0


class CanTransport(Protocol):
    """Synchronous, single-outstanding-operation CAN transport."""

    stats: TransportStats

    def open(self) -> None: ...

    def close(self) -> None: ...

    @property
    def info(self) -> dict[str, str | int]: ...

    def send(self, channel: int, identifier: int, data: bytes) -> None: ...

    def receive(self, channel: int, timeout: float) -> list[CanFrame]: ...
