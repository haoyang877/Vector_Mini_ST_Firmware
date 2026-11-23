"""核对USB-CAN设备选择和资源释放；SDK完全由替身提供，不访问硬件。"""

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from host_app.ports.controlcanfd import ControlCanFd


class ControlCanFdSelectionTests(unittest.TestCase):
    """设备索引必须传给打开函数，不能静默退回另一只适配器。"""

    def test_device_one_channel_zero_and_close(self) -> None:
        """选择第二只设备，通道仍为零；退出只释放当前句柄。"""
        names = (
            "OpenDevice",
            "InitCAN",
            "SetAbitBaud",
            "SetDbitBaud",
            "SetCANFDStandard",
            "StartCAN",
            "ResetCAN",
            "CloseDevice",
            "TransmitFD",
            "ReceiveFD",
        )
        dll = SimpleNamespace(**{"ZCAN_" + name: Mock(return_value=1) for name in names})
        dll.ZCAN_OpenDevice.return_value = 11
        dll.ZCAN_InitCAN.return_value = 22
        with (
            patch.object(Path, "is_file", return_value=True),
            patch("host_app.ports.controlcanfd.C.WinDLL", return_value=dll, create=True),
        ):
            with ControlCanFd(Path("test-sdk.dll"), device_index=1) as port:
                self.assertEqual((port.device, port.channel), (11, 22))
            self.assertIsNone(port.device)
        dll.ZCAN_OpenDevice.assert_called_once_with(41, 1, 0)
        self.assertEqual(dll.ZCAN_InitCAN.call_args.args[:2], (11, 0))
        dll.ZCAN_SetAbitBaud.assert_called_once_with(11, 0, 1000000)
        dll.ZCAN_SetDbitBaud.assert_called_once_with(11, 0, 5000000)
        dll.ZCAN_ResetCAN.assert_called_once_with(22)
        dll.ZCAN_CloseDevice.assert_called_once_with(11)

    def test_desktop_channel_two_and_normal_fd_transmission(self) -> None:
        """GUI 第二通道在 SDK 中为 1；普通数据帧必须保留 CAN 控制器自动重发。"""
        names = (
            "OpenDevice",
            "InitCAN",
            "SetAbitBaud",
            "SetDbitBaud",
            "SetCANFDStandard",
            "StartCAN",
            "ResetCAN",
            "CloseDevice",
            "TransmitFD",
            "ReceiveFD",
        )
        dll = SimpleNamespace(**{"ZCAN_" + name: Mock(return_value=1) for name in names})
        with (
            patch.object(Path, "is_file", return_value=True),
            patch("host_app.ports.controlcanfd.C.WinDLL", return_value=dll, create=True),
            ControlCanFd(
                Path("test.dll"), channel_index=1, nominal_bps=1000000, data_bps=5000000
            ) as port,
        ):
            port.send(0x0CEF0302, bytes(20))
        dll.ZCAN_SetAbitBaud.assert_called_once_with(1, 1, 1000000)
        dll.ZCAN_SetDbitBaud.assert_called_once_with(1, 1, 5000000)
        self.assertEqual(dll.ZCAN_InitCAN.call_args.args[:2], (1, 1))
        tx = dll.ZCAN_TransmitFD.call_args.args[1]._obj
        self.assertEqual((tx.transmit_type, tx.frame.eff, tx.frame.brs), (0, 1, 1))

    def test_link_tool_arbitration_alias(self) -> None:
        """两种工具的仲裁速率名称指向同一配置，同时传入不同值时拒绝打开。"""
        port = ControlCanFd(Path("test.dll"), arb_bps=500_000, data_bps=3_000_000)
        self.assertEqual(
            (port.nominal_bps, port.arb_bps, port.data_bps), (500_000, 500_000, 3_000_000)
        )
        with self.assertRaises(ValueError):
            ControlCanFd(Path("test.dll"), nominal_bps=1_000_000, arb_bps=500_000)

    def test_invalid_index_rejected_before_open(self) -> None:
        """负数、越界和隐式类型转换都不能选择设备。"""
        for value in (-1, 0x100000000, True, 1.5):
            with self.subTest(value=value), self.assertRaises(ValueError):
                ControlCanFd(Path("test-sdk.dll"), device_index=value)


if __name__ == "__main__":
    unittest.main()
