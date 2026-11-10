"""双实例嵌套执行实际 APP、控制、标定及协议模块，宿主侧仅替换板级设备。"""

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from _bsp_test_support import ROOT, arguments, run_c


def main() -> None:
    args = arguments(__doc__)
    project = ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx"
    units = [
        (project.parent / Path(item.text.replace("\\", "/"))).resolve()
        for group in ET.parse(project).findall(".//Group")
        if group.findtext("GroupName")
        in {"00_App", "10_Motor", "20_Parameters", "30_Communication", "50_Shared"}
        for item in group.findall("Files/File/FilePath")
        if item.text.endswith(".c")
    ]
    fixture = Path(__file__).with_name("app_instance_test.c").read_text(encoding="utf-8")
    for board in (1, 2):
        flags = (
            ["-DAPP_TARGET_AT32_F456=1", "-I", ROOT / "shared/targets/at32f456_bringup"]
            if board == 2
            else []
        )
        run_c(
            args,
            f"instances_{board}",
            fixture,
            board,
            [
                *flags,
                *units,
                ROOT / "tests/unit/yg_protocol_bsp_stubs.c",
                ROOT / "tests/unit/bsp_preemption_stubs.c",
            ],
        )

        vector = json.loads(
            (ROOT / "tests/fixtures/protocol/controller_services_v1.json").read_text(
                encoding="utf-8"
            )
        )
        log = (args.out / f"instances_{board}.log").read_text(encoding="utf-8")
        if f"COORDINATE_CRC {vector['coordinate_crc_v1_hex']}" not in log.splitlines():
            raise RuntimeError("坐标CRC与独立zlib黄金向量不一致")


if __name__ == "__main__":
    main()
