"""ControlCANFD窄适配端口；结构依据已有已验证台架SDK契约，导入不访问设备。"""

from __future__ import annotations

import ctypes as C
import time
from pathlib import Path


class _CANFD_INIT(C.Structure):
    _fields_ = [
        ("acc_code", C.c_uint),
        ("acc_mask", C.c_uint),
        ("abit_timing", C.c_uint),
        ("dbit_timing", C.c_uint),
        ("brp", C.c_uint),
        ("filter", C.c_ubyte),
        ("mode", C.c_ubyte),
        ("pad", C.c_ushort),
        ("reserved", C.c_uint),
    ]


class _CHANNEL_INIT(C.Union):
    _fields_ = [("canfd", _CANFD_INIT)]


class _CHANNEL_CONFIG(C.Structure):
    _fields_ = [("can_type", C.c_uint), ("config", _CHANNEL_INIT)]


class _CANFD_FRAME(C.Structure):
    _fields_ = [
        ("can_id", C.c_uint32, 29),
        ("err", C.c_uint32, 1),
        ("rtr", C.c_uint32, 1),
        ("eff", C.c_uint32, 1),
        ("length", C.c_ubyte),
        ("brs", C.c_ubyte, 1),
        ("esi", C.c_ubyte, 1),
        ("reserved", C.c_ubyte, 6),
        ("reserved0", C.c_ubyte),
        ("reserved1", C.c_ubyte),
        ("data", C.c_ubyte * 64),
    ]


class _TXFD(C.Structure):
    _fields_ = [("frame", _CANFD_FRAME), ("transmit_type", C.c_uint)]


class _RXFD(C.Structure):
    _fields_ = [("frame", _CANFD_FRAME), ("timestamp", C.c_ulonglong)]


class ControlCanFd:
    """仅在显式上下文内打开指定设备的通道0，ISO CAN FD 1M/5M。"""

    def __init__(self, dll_path: Path, device_index: int = 0) -> None:
        if type(device_index) is not int or not 0 <= device_index <= 0xFFFFFFFF:
            raise ValueError("USB-CAN设备索引必须是非负u32整数")
        self.dll_path = dll_path.resolve()
        self.device_index = device_index
        self.device = None
        self.channel = None
        self.pending: list[tuple[int, bytes]] = []

    def __enter__(self) -> ControlCanFd:
        """加载用户指定SDK并打开唯一台架通道；任一步失败关闭本次设备。"""
        if not self.dll_path.is_file():
            raise OSError("未找到ControlCANFD DLL")
        self.dll = C.WinDLL(str(self.dll_path))
        self.dll.ZCAN_OpenDevice.argtypes = (C.c_uint, C.c_uint, C.c_uint)
        self.dll.ZCAN_OpenDevice.restype = C.c_void_p
        self.dll.ZCAN_InitCAN.argtypes = (C.c_void_p, C.c_uint, C.c_void_p)
        self.dll.ZCAN_InitCAN.restype = C.c_void_p
        for name in ("ZCAN_SetAbitBaud", "ZCAN_SetDbitBaud", "ZCAN_SetCANFDStandard"):
            getattr(self.dll, name).argtypes = (C.c_void_p, C.c_uint, C.c_uint)
        for name in ("ZCAN_StartCAN", "ZCAN_ResetCAN", "ZCAN_CloseDevice"):
            getattr(self.dll, name).argtypes = (C.c_void_p,)
        self.dll.ZCAN_TransmitFD.argtypes = (C.c_void_p, C.c_void_p, C.c_uint)
        self.dll.ZCAN_ReceiveFD.argtypes = (C.c_void_p, C.c_void_p, C.c_uint, C.c_int)
        self.device = self.dll.ZCAN_OpenDevice(41, self.device_index, 0)
        if not self.device:
            raise OSError("CAN适配器打开失败，检查设备独占与驱动")
        try:
            for name, value in (
                ("ZCAN_SetAbitBaud", 1000000),
                ("ZCAN_SetDbitBaud", 5000000),
                ("ZCAN_SetCANFDStandard", 0),
            ):
                if getattr(self.dll, name)(self.device, 0, value) != 1:
                    raise OSError(name + "失败")
            config = _CHANNEL_CONFIG()
            config.can_type = 1
            self.channel = self.dll.ZCAN_InitCAN(self.device, 0, C.byref(config))
            if not self.channel or self.dll.ZCAN_StartCAN(self.channel) != 1:
                raise OSError("CAN通道初始化或启动失败")
            return self
        except BaseException:
            self.__exit__(None, None, None)
            raise

    def __exit__(self, *exception: object) -> None:
        """关闭本次会话；不执行电机控制、复位目标MCU或恢复运动。"""
        try:
            if self.channel:
                self.dll.ZCAN_ResetCAN(self.channel)
        finally:
            if self.device:
                self.dll.ZCAN_CloseDevice(self.device)
            self.channel = self.device = None

    def send(self, identifier: int, data: bytes) -> None:
        """发送扩展CAN FD+BRS帧，SDK失败立即停止事务。"""
        if not self.channel or len(data) not in (20, 24, 32, 48, 64):
            raise OSError("CAN通道未打开或帧长度非法")
        tx = _TXFD()
        tx.frame.can_id = identifier
        tx.frame.eff = tx.frame.brs = 1
        tx.frame.length = len(data)
        tx.frame.data[: len(data)] = data
        if self.dll.ZCAN_TransmitFD(self.channel, C.byref(tx), 1) != 1:
            raise OSError("CAN FD发送失败；不重开设备或继续擦写")

    def receive(self, timeout_s: float) -> tuple[int, bytes] | None:
        """接收有界批次；总线错误或ESI立即失败，不将错误帧当作有效业务回复。"""
        deadline = time.monotonic() + timeout_s
        while self.channel:
            if self.pending:
                return self.pending.pop(0)
            batch = (_RXFD * 32)()
            count = self.dll.ZCAN_ReceiveFD(self.channel, C.byref(batch), 32, 0)
            if not 0 <= count <= 32:
                raise OSError("CAN FD接收失败")
            for item in batch[:count]:
                frame = item.frame
                if frame.err or frame.esi:
                    raise OSError("CAN总线错误或设备error-passive；停止升级")
                if frame.length > 64:
                    raise OSError("SDK返回非法帧长度")
                if frame.eff and frame.brs and not frame.rtr:
                    self.pending.append((frame.can_id, bytes(frame.data[: frame.length])))
            if self.pending:
                continue
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.001)
        raise OSError("CAN通道未打开")
