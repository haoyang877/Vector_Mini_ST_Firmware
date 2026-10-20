"""从单一目标输入生成Loader Keil输入和共享内存常量；不构建、不访问设备。"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT

sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target, repository_path  # noqa: E402


def main() -> None:
    """将明确目标生成到源码目录，产物输出目录保持outputs下。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="目标名称或目标 JSON；省略保持 G431")
    args = parser.parse_args()
    config = load_target(args.target)
    build = config["build"]
    target_dir = repository_path(ROOT, build["target_directory"])
    target_header = repository_path(ROOT, build["target_header"])
    constants = config["constants"]
    header = [
        f"#ifndef {config['target'].upper()}_H",
        f"#define {config['target'].upper()}_H",
        "#include <stdint.h>",
        "/* 由目标清单生成；禁止手工修改。 */",
    ]
    header += [f"#define {key} {value}" for key, value in constants.items()]
    header += ["#endif", ""]
    target_header.write_text("\n".join(header), encoding="utf-8")
    flash = int(constants["TARGET_FLASH_BASE"].rstrip("U"), 0)
    size = int(constants["TARGET_LOADER_BYTES"].rstrip("U"), 0)
    ram = int(constants["TARGET_RAM_BASE"].rstrip("U"), 0)
    end = int(constants["TARGET_RAM_END"].rstrip("U"), 0)
    scatter = f"""; 由单一目标生成；保留RAM顶32B邮箱，不初始化、不分配到普通RW。
LR_LOADER 0x{flash:08X} 0x{size:X}
{{
    ER_LOADER 0x{flash:08X} 0x{size:X}
    {{
        *.o (RESET, +First)
        *(InRoot$$Sections)
        .ANY (+RO)
    }}
    RW_LOADER 0x{ram:08X} 0x{end - ram:X}
    {{
        .ANY (+RW +ZI)
    }}
}}
"""
    target_dir.mkdir(parents=True, exist_ok=True)
    selection_dir = repository_path(ROOT, build["selection_directory"])
    selection_dir.mkdir(parents=True, exist_ok=True)
    (selection_dir / "loader_target.h").write_text(
        '/* 构建期选择目标，不在运行时切换。 */\n#include "' + target_header.name + '"\n',
        encoding="utf-8",
    )
    (target_dir / "loader.sct").write_text(scatter, encoding="utf-8")
    app = int(constants["TARGET_APP_BASE"].rstrip("U"), 0)
    app_bytes = int(constants["TARGET_APP_BYTES"].rstrip("U"), 0)
    descriptor = int(constants["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
    app_scatter = f"""; APP专用Loader布局，工程必须同时编入app_descriptor.c和显式版本/角色宏。
LR_APP 0x{app:08X} 0x{app_bytes:X}
{{
    ER_VECTORS 0x{app:08X} 0x{descriptor:X} {{ *.o (RESET, +First) }}
    ER_DESCRIPTOR 0x{app + descriptor:08X} FIXED 0x60 {{ * (.app_descriptor) }}
    ER_APP 0x{app + descriptor + 96:08X} FIXED 0x{app_bytes - descriptor - 96:X}
    {{
        *(InRoot$$Sections)
        .ANY (+RO)
    }}
    RW_APP 0x{ram:08X} 0x{end - ram:X} {{ .ANY (+RW +ZI) }}
}}
"""
    (target_dir / "app.sct").write_text(app_scatter, encoding="utf-8")
    tree = ET.parse(repository_path(ROOT, build["template"]))
    targets = tree.getroot().find("Targets")
    target = targets.find("Target")
    for other in list(targets)[1:]:
        targets.remove(other)

    def set_value(path: str, value: str) -> None:
        """更新已知Keil模板节点，缺节点时报错而不静默产生残缺工程。"""
        item = target.find(path)
        if item is None:
            raise ValueError(f"缺少Keil模板节点：{path}")
        item.text = value

    def relative(path: str) -> str:
        """生成相对于独立工程的路径，不写入工作站绝对位置。"""
        return os.path.relpath(ROOT / path, target_dir).replace("/", "\\")

    set_value("TargetName", config["name"])
    common = "TargetOption/TargetCommonOption/"
    output = relative(build["loader_output"]) + "\\"
    for key, value in {
        "Device": config["device"],
        "OutputName": config["name"],
        "OutputDirectory": output,
        "ListingPath": output,
        "Cpu": f"IRAM(0x{ram:X}-0x{end - 1:X}) IROM(0x{flash:X}-0x{flash + size - 1:X}) {build['cpu']}",
    }.items():
        set_value(common + key, value)
    for group in ("BeforeCompile", "BeforeMake", "AfterMake"):
        for child in target.find(common + group):
            child.text = (
                "0"
                if child.tag.startswith(("Run", "nStop", "UserProg1Dos", "UserProg2Dos"))
                else ""
            )
    # 不从APP模板继承自动下载、调试器选择或工作站Flash驱动路径。
    utilities = target.find("TargetOption/Utilities")
    if utilities is not None:
        target.find("TargetOption").remove(utilities)
    ads = "TargetOption/TargetArmAds/"
    set_value(ads + "Cads/VariousControls/Define", build["defines"])
    set_value(
        ads + "Cads/VariousControls/IncludePath", ";".join(relative(p) for p in config["includes"])
    )
    set_value(ads + "Cads/VariousControls/MiscControls", "")
    set_value(ads + "Cads/Optim", "4")
    # Loader无stdio/堆分配，选择ARM MicroLIB；APP沿用原运行库与栈预算。
    set_value(ads + "ArmAdsMisc/useUlib", "1")
    set_value(ads + "LDads/umfTarg", "0")
    set_value(ads + "LDads/useFile", "1")
    set_value(ads + "LDads/ScatterFile", "loader.sct")
    memories = target.find(ads + "ArmAdsMisc/OnChipMemories")
    for item in memories:
        start = item.find("StartAddress")
        length = item.find("Size")
        if start is not None and length is not None:
            if item.tag in ("IROM", "OCR_RVCT4"):
                start.text, length.text = hex(flash), hex(size)
            elif item.tag in ("IRAM", "OCR_RVCT9"):
                start.text, length.text = hex(ram), hex(end - ram)
            else:
                start.text, length.text = "0x0", "0x0"
    groups = target.find("Groups")
    groups.clear()
    for name, sources in config["groups"].items():
        group = ET.SubElement(groups, "Group")
        ET.SubElement(group, "GroupName").text = name
        files = ET.SubElement(group, "Files")
        for source in sources:
            item = ET.SubElement(files, "File")
            ET.SubElement(item, "FileName").text = Path(source).name
            ET.SubElement(item, "FileType").text = "2" if source.endswith(".s") else "1"
            ET.SubElement(item, "FilePath").text = relative(source)
    # 模板的 RTE/层名称也归 Loader，不能继承普通 APP 的工程身份。
    for info in tree.getroot().findall("RTE/packages/package/targetInfos/targetInfo"):
        info.set("name", config["name"])
    layer = tree.getroot().find("LayerInfo/Layers/Layer/LayName")
    if layer is not None:
        layer.text = config["name"]
    ET.indent(tree, space="  ")
    tree.write(
        repository_path(ROOT, build["loader_project"]), encoding="utf-8", xml_declaration=True
    )


if __name__ == "__main__":
    main()
