"""显式桌面离线检查：导航、绑定准入、任务互斥、窄窗口，不加载DLL或打开设备。"""

import argparse
import json
from pathlib import Path
import sys
import struct
import threading
import time
import tkinter as tk
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from loader_flasher_gui import FlasherWindow, find_vendor_dll  # noqa: E402
from maintenance_version import VERSION_TEXT  # noqa: E402


def require(value, message):
    """离线界面准入检查使用显式异常。"""
    if not value:
        raise RuntimeError(message)


def main():
    """显式运行Tk检查；硬件函数被替身禁止调用。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    report = {"status": "FAIL", "hardware_accessed": False}
    root = tk.Tk()
    gate = threading.Event()
    try:
        with (
            patch(
                "loader_flasher_gui.open_probe",
                side_effect=RuntimeError("Unexpected hardware"),
            ),
            patch(
                "loader_flasher_gui.can_snapshot",
                side_effect=RuntimeError("Unexpected hardware"),
            ),
            patch(
                "loader_flasher_gui.flash_snapshot",
                side_effect=RuntimeError("Unexpected hardware"),
            ),
        ):
            window = FlasherWindow(root)
            root.geometry("940x740+10+10")
            root.update()
            require(f"v{VERSION_TEXT}" in root.title(), "Software version missing")
            require(
                str(window.flash_button["state"]) == "disabled",
                "Default Flash button is enabled",
            )
            require(
                str(window.upgrade_button["state"]) == "disabled",
                "Default upgrade button is enabled",
            )
            pages = []
            for index, button in enumerate(window.nav_buttons):
                button.invoke()
                root.update()
                require(
                    window.tabs.index(window.tabs.select()) == index,
                    "Sidebar selects wrong page",
                )
                pages.append(window.tabs.tab(index, "text"))
                write = (
                    window.flash_button
                    if index == 0
                    else window.upgrade_button
                    if index == 1
                    else None
                )
                if write is not None:
                    require(
                        write.winfo_rooty() + write.winfo_height()
                        <= root.winfo_rooty() + root.winfo_height(),
                        "Primary action is outside narrow window",
                    )
            window.toggle_logs()
            require(window.logs_visible, "Log panel failed to open")
            window.toggle_logs()
            require(not window.logs_visible, "Log panel failed to close")
            # SWD替身没有写入/复位方法，验证显示取自板内ID而非CAN下拉框。
            raw_node = struct.pack("<f", 0)
            probe = SimpleNamespace(
                open=lambda **kwargs: None,
                exec_command=lambda text: None,
                set_tif=lambda value: None,
                connect=lambda *args, **kwargs: None,
                halted=lambda: False,
                memory_read16=lambda address, count: [128],
                memory_read32=lambda address, count: [0x468]
                if count == 1
                else [1, 2, 3],
                memory_read8=lambda address, count: list(raw_node),
                hardware_status=SimpleNamespace(VTarget=3300),
                close=lambda: None,
            )
            fake_pylink = SimpleNamespace(
                enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1))
            )
            with (
                patch("loader_flasher_gui.open_probe", return_value=probe),
                patch("loader_flasher_gui.check_power_off", return_value=[]),
                patch.dict(sys.modules, {"pylink": fake_pylink}),
                patch.object(
                    window,
                    "start_job",
                    side_effect=lambda label, work, done: done(work()),
                ),
            ):
                window.probe_serial.set("12345")
                window.can_node.set("0")
                for node, name in enumerate(
                    ("默认电机", "左轮", "右轮", "roll", "pitch", "yaw", "lift")
                ):
                    raw_node = struct.pack("<f", node)
                    window.read_target()
                    require(
                        f"{name}（ID {node}）" in window.target_info.get(),
                        "Wrong motor identity",
                    )
                raw_node = b"\xff" * 4
                window.read_target()
                require(
                    "未配置" in window.target_info.get()
                    and "默认电机" not in window.target_info.get(),
                    "Blank parameters shown as ID0",
                )
                window.tabs.select(0)
                root.update()
                require(
                    window.flash_button.winfo_rooty()
                    + window.flash_button.winfo_height()
                    <= root.winfo_rooty() + root.winfo_height(),
                    "Read target pushes action outside window",
                )
                window.clear_target()
                require(
                    window.target_info.get() == "尚未绑定J-Link目标",
                    "Stale motor identity after probe change",
                )
                raw_node = struct.pack("<f", 2)
                window.read_target()
                window.firmware = {"hex_sha256": "offline"}
                window.hex_path.set("offline.hex")
                with patch(
                    "loader_flasher_gui.flash_package",
                    return_value={"cpu_running": True, "uid": window.binding["uid"]},
                ):
                    window.flash()
                require(
                    "右轮（ID 2）" in window.target_info.get(),
                    "Motor name lost after successful flash",
                )
                require(
                    window.binding is None
                    and str(window.flash_button["state"]) == "disabled",
                    "Flash success retained write authorization",
                )
            window.start_job("离线任务", lambda: gate.wait(3), lambda _: None)
            root.update()
            require(window.busy, "Task did not lock controls")
            require(
                all(
                    str(widget["state"]) == "disabled" for widget, _ in window.controls
                ),
                "Inputs remain enabled during operation",
            )
            with patch("loader_flasher_gui.messagebox.showinfo") as notice:
                window.close()
                require(
                    notice.called and root.winfo_exists(), "Busy close was not rejected"
                )
            gate.set()
            deadline = time.monotonic() + 4
            while window.busy and time.monotonic() < deadline:
                root.update()
                time.sleep(0.02)
            require(not window.busy, "Task failed to unlock")
            require(
                all(str(widget["state"]) == state for widget, state in window.controls),
                "Input states were not restored",
            )
            require(
                str(window.flash_button["state"]) == "disabled",
                "Unbound Flash became enabled after task",
            )
            driver_root = args.out / "fake_bundle"
            (driver_root / "drivers").mkdir(parents=True, exist_ok=True)
            (driver_root / "drivers/ControlCANFD.dll").write_bytes(
                b"offline path fixture"
            )
            with patch.object(sys, "_MEIPASS", str(driver_root), create=True):
                require(
                    find_vendor_dll("ControlCANFD.dll")
                    == str(driver_root / "drivers/ControlCANFD.dll"),
                    "Embedded driver not preferred",
                )
            report.update(
                status="PASS",
                pages=pages,
                sidebar_checked=True,
                mutual_exclusion_checked=True,
                busy_close_checked=True,
                narrow_window="940x740",
                embedded_driver_lookup_checked=True,
                motor_id_display_checked=True,
            )
    finally:
        gate.set()
        root.destroy()
        (args.out / "summary.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
