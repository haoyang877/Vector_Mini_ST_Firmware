"""Chuangxin USBCANFD-200U transport using the ControlCANFD DLL ABI.

One instance owns one configured channel. The vendor API and this wrapper are
intended for single-threaded use.
"""

from __future__ import annotations

import ctypes as C
from dataclasses import dataclass, field
import hashlib
import os
from pathlib import Path
import time

from .base import TransportError, TransportStats
from .frame import CanFrame


DEFAULT_DLL = Path("D:/Work/Code/motor_ctrl_app/.local/canfd-sdk/ControlCANFD.dll")


def _default_dll_path() -> Path:
    return Path(os.environ.get("MDRIVE_CAN_DLL", DEFAULT_DLL))


@dataclass(frozen=True, slots=True)
class ControlCanFdConfig:
    """ControlCANFD adapter configuration."""

    dll_path: Path = field(default_factory=_default_dll_path)
    channel: int = 0
    nominal_bps: int = 1_000_000
    data_bps: int = 1_000_000
    iso: bool = True


class FDFrame(C.Structure):
    _fields_ = [
        ("can_id", C.c_uint32),
        ("length", C.c_uint8),
        ("flags", C.c_uint8),
        ("reserved", C.c_uint8 * 2),
        ("data", C.c_uint8 * 64),
    ]


class CANFrame(C.Structure):
    _fields_ = [
        ("can_id", C.c_uint32),
        ("length", C.c_uint8),
        ("reserved", C.c_uint8 * 3),
        ("data", C.c_uint8 * 8),
    ]


class TxFD(C.Structure):
    _fields_ = [("frame", FDFrame), ("transmit_type", C.c_uint32)]


class RxFD(C.Structure):
    _fields_ = [("frame", FDFrame), ("timestamp", C.c_uint64)]


class RxCAN(C.Structure):
    _fields_ = [("frame", CANFrame), ("timestamp", C.c_uint64)]


class FDConfig(C.Structure):
    _fields_ = [
        *((name, C.c_uint32) for name in ("acc_code", "acc_mask", "abit_timing", "dbit_timing", "brp")),
        ("filter", C.c_uint8),
        ("mode", C.c_uint8),
        ("pad", C.c_uint16),
        ("reserved", C.c_uint32),
    ]


class InitConfig(C.Structure):
    _fields_ = [("can_type", C.c_uint32), ("canfd", FDConfig)]


class DeviceInfo(C.Structure):
    _fields_ = [
        *((name, C.c_uint16) for name in ("hw_version", "fw_version", "dr_version", "in_version", "irq_num")),
        ("can_num", C.c_uint8),
        ("serial", C.c_char * 21),
        ("hardware", C.c_char * 40),
        ("reserved", C.c_uint16 * 4),
    ]


class ControlCanFdTransport:
    """Single-channel, single-threaded ControlCANFD transport."""

    def __init__(self, config: ControlCanFdConfig | None = None) -> None:
        self.config = config or ControlCanFdConfig()
        self.stats = TransportStats()
        self._dll = None
        self._device = None
        self._channel_handle = None
        self._info: dict[str, str | int] = {}

    @property
    def info(self) -> dict[str, str | int]:
        return dict(self._info)

    @staticmethod
    def _ok(code: int, operation: str) -> None:
        if code != 1:
            raise TransportError(f"{operation}: SDK returned {code}")

    def _require_open(self, channel: int):
        if channel != self.config.channel:
            raise TransportError(f"transport owns channel {self.config.channel}, not {channel}")
        if self._dll is None or self._channel_handle is None:
            raise TransportError("CAN transport is not open")
        return self._dll, self._channel_handle

    def open(self) -> None:
        """Load the DLL, open device type 41, and start the configured channel."""
        if self._dll is not None:
            return
        path = self.config.dll_path.resolve()
        try:
            dll = C.WinDLL(str(path))
        except (OSError, AttributeError) as exc:
            raise TransportError(f"unable to load ControlCANFD DLL {path}: {exc}") from exc
        self._bind(dll)
        self._dll = dll
        try:
            # The USBCANFD occasionally refuses to open for a short window
            # (observed after heavy session churn); retry before giving up.
            device = None
            for _ in range(5):
                device = dll.ZCAN_OpenDevice(41, 0, 0)
                if device:
                    break
                time.sleep(0.3)
            self._device = device
            if not self._device:
                raise TransportError("USBCANFD-200U open failed (5 attempts)")
            info = DeviceInfo()
            self._ok(dll.ZCAN_GetDeviceInf(self._device, C.byref(info)), "device info")
            channel = self.config.channel
            self._ok(dll.ZCAN_SetAbitBaud(self._device, channel, self.config.nominal_bps), "nominal bitrate")
            self._ok(dll.ZCAN_SetDbitBaud(self._device, channel, self.config.data_bps), "data bitrate")
            self._ok(dll.ZCAN_SetCANFDStandard(self._device, channel, 0 if self.config.iso else 1), "CAN-FD standard")
            config = InitConfig()
            config.can_type = 1
            self._channel_handle = dll.ZCAN_InitCAN(self._device, channel, C.byref(config))
            if not self._channel_handle:
                raise TransportError("CAN init failed")
            self._ok(dll.ZCAN_StartCAN(self._channel_handle), "start CAN")
            self._info = {
                "serial": info.serial.decode("ascii", "replace").rstrip("\x00"),
                "hardware": info.hardware.decode("ascii", "replace").rstrip("\x00"),
                "hardware_version": info.hw_version,
                "firmware_version": info.fw_version,
                "channels": info.can_num,
                "nominal_bps": self.config.nominal_bps,
                "data_bps": self.config.data_bps,
                "dll_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
        except (OSError, ValueError) as exc:
            try:
                self.close()
            except OSError as close_error:
                raise TransportError(f"{exc}; cleanup failed: {close_error}") from exc
            raise

    @staticmethod
    def _bind(dll) -> None:
        specs = {
            "ZCAN_OpenDevice": ([C.c_uint32] * 3, C.c_void_p),
            "ZCAN_CloseDevice": ([C.c_void_p], C.c_uint32),
            "ZCAN_GetDeviceInf": ([C.c_void_p, C.POINTER(DeviceInfo)], C.c_uint32),
            "ZCAN_InitCAN": ([C.c_void_p, C.c_uint32, C.POINTER(InitConfig)], C.c_void_p),
            "ZCAN_TransmitFD": ([C.c_void_p, C.POINTER(TxFD), C.c_uint32], C.c_uint32),
            "ZCAN_ReceiveFD": ([C.c_void_p, C.POINTER(RxFD), C.c_uint32, C.c_int32], C.c_uint32),
            "ZCAN_Receive": ([C.c_void_p, C.POINTER(RxCAN), C.c_uint32, C.c_int32], C.c_uint32),
        }
        for name in ("ZCAN_SetAbitBaud", "ZCAN_SetDbitBaud", "ZCAN_SetCANFDStandard"):
            specs[name] = ([C.c_void_p, C.c_uint32, C.c_uint32], C.c_uint32)
        for name in ("ZCAN_StartCAN", "ZCAN_ResetCAN"):
            specs[name] = ([C.c_void_p], C.c_uint32)
        for name, (arguments, result) in specs.items():
            function = getattr(dll, name)
            function.argtypes = arguments
            function.restype = result

    def close(self) -> None:
        """Reset the channel and close the device, attempting both operations."""
        errors: list[str] = []
        if self._dll is not None and self._channel_handle is not None:
            try:
                self._ok(self._dll.ZCAN_ResetCAN(self._channel_handle), f"reset CAN{self.config.channel}")
            except OSError as exc:
                errors.append(repr(exc))
            self._channel_handle = None
        if self._dll is not None and self._device:
            try:
                self._ok(self._dll.ZCAN_CloseDevice(self._device), "close device")
            except OSError as exc:
                errors.append(repr(exc))
        self._device = None
        self._dll = None
        if errors:
            raise TransportError("; ".join(errors))

    def send(self, channel: int, identifier: int, data: bytes) -> None:
        dll, handle = self._require_open(channel)
        if not 0 <= identifier <= 0x7FF:
            raise TransportError(f"not a standard 11-bit CAN ID: {identifier:#x}")
        if len(data) > 64:
            raise TransportError(f"CAN-FD payload too long: {len(data)}")
        message = TxFD()
        message.frame.can_id = identifier
        message.frame.length = len(data)
        message.frame.flags = 1
        message.frame.data[: len(data)] = data
        self._ok(dll.ZCAN_TransmitFD(handle, C.byref(message), 1), "transmit FD")
        self.stats.tx_count += 1

    def receive(self, channel: int, timeout: float) -> list[CanFrame]:
        dll, handle = self._require_open(channel)
        if timeout < 0:
            raise TransportError(f"negative receive timeout: {timeout}")
        frames: list[CanFrame] = []
        calls = (
            (RxFD, "ZCAN_ReceiveFD", 64, max(0, round(timeout * 1000))),
            (RxCAN, "ZCAN_Receive", 8, 0),
        )
        for frame_type, name, payload_limit, wait_ms in calls:
            messages = (frame_type * 64)()
            count = getattr(dll, name)(handle, messages, 64, wait_ms)
            if count > 64:
                raise TransportError(f"{name} failed: SDK returned {count}")
            for message in messages[:count]:
                length = message.frame.length
                if length > payload_limit:
                    raise TransportError(f"{name} returned invalid length {length}")
                flags = message.frame.flags if frame_type is RxFD else 0
                frame = CanFrame(message.frame.can_id, bytes(message.frame.data[:length]), flags, message.timestamp / 1_000_000)
                frames.append(frame)
                self.stats.rx_count += 1
                if frame.identifier & 0x20000000:
                    self.stats.error_frames += 1
        return frames


ControlCanFD = ControlCanFdTransport
ControlCANFDConfig = ControlCanFdConfig
ControlCANFDTransport = ControlCanFdTransport
CanFD = ControlCanFdTransport
LoaderBus = ControlCanFdTransport
