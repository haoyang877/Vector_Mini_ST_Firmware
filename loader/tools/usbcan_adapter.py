"""创芯ControlCANFD边界；导入无硬件动作，构造时明确选择适配器与通道。"""

import ctypes as C
import hashlib
from pathlib import Path


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
        (n, C.c_uint32)
        for n in ("acc_code", "acc_mask", "abit_timing", "dbit_timing", "brp")
    ] + [
        ("filter", C.c_uint8),
        ("mode", C.c_uint8),
        ("pad", C.c_uint16),
        ("reserved", C.c_uint32),
    ]


class InitConfig(C.Structure):
    # The FD member is the largest member of the vendor's anonymous union.
    _fields_ = [("can_type", C.c_uint32), ("canfd", FDConfig)]


class DeviceInfo(C.Structure):
    _fields_ = [
        (n, C.c_uint16)
        for n in ("hw_version", "fw_version", "dr_version", "in_version", "irq_num")
    ] + [
        ("can_num", C.c_uint8),
        ("serial", C.c_char * 21),
        ("hardware", C.c_char * 40),
        ("reserved", C.c_uint16 * 4),
    ]


class CanFD:
    """拥有有界的1M/1M ISO CAN-FD通道。"""

    def __init__(self, dll_path, channels=(0,), device_index=0, expected_serial=None):
        if not channels or any(x not in (0, 1) for x in channels):
            raise ValueError("explicit channel 0 or 1 required")
        self.channels = {}
        self.device = None
        path = Path(dll_path).resolve()
        self.dll = C.WinDLL(str(path))
        specs = {
            "ZCAN_OpenDevice": ([C.c_uint32] * 3, C.c_void_p),
            "ZCAN_CloseDevice": ([C.c_void_p], C.c_uint32),
            "ZCAN_GetDeviceInf": ([C.c_void_p, C.POINTER(DeviceInfo)], C.c_uint32),
            "ZCAN_InitCAN": (
                [C.c_void_p, C.c_uint32, C.POINTER(InitConfig)],
                C.c_void_p,
            ),
            "ZCAN_TransmitFD": ([C.c_void_p, C.POINTER(TxFD), C.c_uint32], C.c_uint32),
            "ZCAN_ReceiveFD": (
                [C.c_void_p, C.POINTER(RxFD), C.c_uint32, C.c_int32],
                C.c_uint32,
            ),
            "ZCAN_Receive": (
                [C.c_void_p, C.POINTER(RxCAN), C.c_uint32, C.c_int32],
                C.c_uint32,
            ),
        }
        for n in ("ZCAN_SetAbitBaud", "ZCAN_SetDbitBaud", "ZCAN_SetCANFDStandard"):
            specs[n] = ([C.c_void_p, C.c_uint32, C.c_uint32], C.c_uint32)
        for n in ("ZCAN_StartCAN", "ZCAN_ResetCAN"):
            specs[n] = ([C.c_void_p], C.c_uint32)
        for name, (args, result) in specs.items():
            fn = getattr(self.dll, name)
            fn.argtypes = args
            fn.restype = result
        try:
            self.device = self.dll.ZCAN_OpenDevice(41, device_index, 0)
            if not self.device:
                raise OSError("USBCANFD_200U open failed")
            info = DeviceInfo()
            self._ok(
                self.dll.ZCAN_GetDeviceInf(self.device, C.byref(info)), "device info"
            )
            self.info = {
                "serial": info.serial.decode("ascii", "replace"),
                "hardware": info.hardware.decode("ascii", "replace"),
                "hardware_version": info.hw_version,
                "firmware_version": info.fw_version,
                "channels": info.can_num,
                "nominal_bps": 1000000,
                "data_bps": 1000000,
                "dll_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            }
            if expected_serial is not None and self.info["serial"] != expected_serial:
                raise RuntimeError("Adapter serial mismatch")
            for ch in channels:
                self._ok(
                    self.dll.ZCAN_SetAbitBaud(self.device, ch, 1000000),
                    "nominal bitrate",
                )
                self._ok(
                    self.dll.ZCAN_SetDbitBaud(self.device, ch, 1000000), "data bitrate"
                )
                self._ok(self.dll.ZCAN_SetCANFDStandard(self.device, ch, 0), "ISO FD")
                config = InitConfig()
                config.can_type = 1
                handle = self.dll.ZCAN_InitCAN(self.device, ch, C.byref(config))
                if not handle:
                    raise OSError("CAN init failed")
                self.channels[ch] = handle
                self._ok(self.dll.ZCAN_StartCAN(handle), "start CAN")
        except BaseException:
            self.close()
            raise

    @staticmethod
    def _ok(code, operation):
        if code != 1:
            raise OSError(f"{operation}: SDK returned {code}")

    def close(self):
        """尝试释放全部自己拥有的通道和适配器，错误集中报告。"""
        errors = []
        for ch, handle in self.channels.items():
            try:
                self._ok(self.dll.ZCAN_ResetCAN(handle), f"reset CAN{ch}")
            except Exception as exc:
                errors.append(repr(exc))
        self.channels.clear()
        if self.device:
            try:
                self._ok(self.dll.ZCAN_CloseDevice(self.device), "close device")
            except Exception as exc:
                errors.append(repr(exc))
            self.device = None
        if errors:
            raise OSError("; ".join(errors))
