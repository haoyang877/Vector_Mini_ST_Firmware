# Windows 单文件构建。以工具代码中的版本生成文件名和 Windows 版本资源。
import os
import sys
from pathlib import Path

from PyInstaller.utils.win32.versioninfo import (
    FixedFileInfo, StringFileInfo, StringStruct, StringTable,
    VarFileInfo, VarStruct, VSVersionInfo,
)

root = Path(SPECPATH).resolve().parents[1]
bench = root / "tools/bench"
sys.path.insert(0, str(bench))
from can_link_test import TOOL_VERSION

name = f"CAN-Link-Test-v{TOOL_VERSION}"
version = (*map(int, TOOL_VERSION.split(".")), 0)
sdk_root = Path(os.environ.get("MOTOR_CTRL_APP_ROOT", root.parent / "motor_ctrl_app"))
dll = sdk_root / ".local/canfd-sdk/ControlCANFD.dll"
resource = VSVersionInfo(
    FixedFileInfo(filevers=version, prodvers=version, OS=0x40004, fileType=1),
    [StringFileInfo([StringTable("040904B0", [
        StringStruct("FileVersion", TOOL_VERSION),
        StringStruct("ProductVersion", TOOL_VERSION),
        StringStruct("ProductName", "CAN Link Test"),
        StringStruct("OriginalFilename", name + ".exe"),
    ])]), VarFileInfo([VarStruct("Translation", [1033, 1200])])],
)
a = Analysis(
    [str(bench / "_can_link_window.py")],
    pathex=[str(root / "host_app"), str(root / "shared/targets")],
    binaries=[(str(dll), ".")],
)
exe = EXE(PYZ(a.pure), a.scripts, a.binaries, a.datas, [], name=name, console=False,
          upx=False, version=resource)
