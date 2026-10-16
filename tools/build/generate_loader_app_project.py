"""从普通APP源清单生成Loader配套Keil工程；只写工程输入，不构建或访问硬件。"""

from __future__ import annotations

import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import KEIL, ROOT


def main() -> None:
    """复用正常控制源码，仅替换启动能力、显式镜像身份和目标内存布局。"""
    config = json.loads(
        (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
    )["constants"]
    tree = ET.parse(KEIL / "YG_ESC_APP.uvprojx")
    target = tree.getroot().find("Targets/Target")
    if target is None:
        raise ValueError("普通APP工程缺少Target")

    def set_value(path: str, value: str) -> None:
        """更新必需的模板字段；缺失时停止，不输出残缺工程。"""
        node = target.find(path)
        if node is None:
            raise ValueError(f"普通APP工程缺少{path}")
        node.text = value

    def relative(path: str) -> str:
        """项目路径相对工程目录，不固化工作站位置。"""
        return os.path.relpath(ROOT / path, KEIL).replace("/", "\\")

    name = "Vector_Mini_ST_LoaderApp"
    set_value("TargetName", name)
    common = "TargetOption/TargetCommonOption/"
    output = relative(f"outputs/build/keil/{name}") + "\\"
    app = int(config["TARGET_APP_BASE"].rstrip("U"), 0)
    size = int(config["TARGET_APP_BYTES"].rstrip("U"), 0)
    ram = int(config["TARGET_RAM_BASE"].rstrip("U"), 0)
    end = int(config["TARGET_RAM_END"].rstrip("U"), 0)
    set_value(common + "OutputName", name)
    set_value(common + "OutputDirectory", output)
    set_value(common + "ListingPath", output)
    set_value(
        common + "Cpu",
        f"IRAM(0x{ram:X}-0x{end - 1:X}) IROM(0x{app:X}-0x{app + size - 1:X}) "
        'CLOCK(8000000) FPU2 CPUTYPE("Cortex-M4")',
    )
    # 新工程不继承普通 APP 的执行钩子；下载驱动沿用 Keil 原生配置，写域由 APP 段限定。
    for hook in ("BeforeCompile", "BeforeMake", "AfterMake"):
        for child in target.findall(common + hook + "/*"):
            child.text = (
                "0"
                if child.tag.startswith(("Run", "nStop", "UserProg1Dos", "UserProg2Dos"))
                else ""
            )
    after = target.find(common + "AfterMake")
    if after is None:
        raise ValueError("普通APP工程缺少AfterMake")
    after.find("RunUserProg1").text = "1"
    after.find("nStopA1X").text = "2"
    after.find("UserProg1Name").text = (
        '"'
        + relative(".venv/Scripts/python.exe")
        + '" "'
        + relative("tools/build/keil_loader_app.py")
        + '" "#L"'
    )
    ads = "TargetOption/TargetArmAds/"
    defines = target.find(ads + "Cads/VariousControls/Define")
    # 配套APP版本来自共同目标清单，禁止运行时与描述符独立赋值。
    defines.text = (defines.text or "") + (
        ",YG_APP_LOADER_IMAGE=1,YG_APP_IMAGE_VERSION=TARGET_APP_VERSION,YG_APP_IMAGE_ROLE=1U"
    )
    set_value(ads + "LDads/umfTarg", "0")
    set_value(ads + "LDads/useFile", "1")
    set_value(ads + "LDads/ScatterFile", relative("loader/targets/vector_mini_st_g431/app.sct"))
    for item in target.find(ads + "ArmAdsMisc/OnChipMemories"):
        start, length = item.find("StartAddress"), item.find("Size")
        if start is not None and length is not None:
            if item.tag in ("IROM", "OCR_RVCT4"):
                start.text, length.text = hex(app), hex(size)
            elif item.tag in ("IRAM", "OCR_RVCT9"):
                start.text, length.text = hex(ram), hex(end - ram)
            else:
                start.text, length.text = "0x0", "0x0"
    groups = target.find("Groups")
    for file in groups.findall("Group/Files/File"):
        if file.findtext("FileName") == "boot_hw_unavailable.c":
            file.find("FileName").text = "boot_hw_stm32g4.c"
            file.find("FilePath").text = relative(
                "firmware/platform/stm32g4/ports/boot/boot_hw_stm32g4.c"
            )
    files = groups.find("Group[GroupName='50_Shared']/Files")
    if files is None:
        raise ValueError("普通APP工程缺少50_Shared分组")
    # APP 只保留进 Loader 的 RAM 邮箱与内嵌描述符；镜像校验与启动判据归 Loader。
    for source in (
        "shared/boot/image_mailbox.c",
        "shared/targets/vector_mini_st_g431_app_descriptor.c",
    ):
        file = ET.SubElement(files, "File")
        ET.SubElement(file, "FileName").text = Path(source).name
        ET.SubElement(file, "FileType").text = "1"
        ET.SubElement(file, "FilePath").text = relative(source)
    for info in tree.getroot().findall("RTE/packages/package/targetInfos/targetInfo"):
        info.set("name", name)
    layer = tree.getroot().find("LayerInfo/Layers/Layer/LayName")
    if layer is not None:
        layer.text = name
    ET.indent(tree, space="  ")
    tree.write(KEIL / f"{name}.uvprojx", encoding="utf-8", xml_declaration=True)


if __name__ == "__main__":
    main()
