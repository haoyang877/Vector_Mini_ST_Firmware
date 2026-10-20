"""从普通APP源清单生成Loader配套Keil工程；只写工程输入，不构建或访问硬件。"""

from __future__ import annotations

import argparse
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import ROOT

sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target, repository_path  # noqa: E402


def render_project(profile: dict, root: Path) -> str:
    """只读生成配套 APP 工程；设备、宏、路径和链接布局均绑定所选目标。"""
    config = profile["constants"]
    build = profile["build"]
    project = repository_path(root, build["app_project"])
    project_dir = project.parent
    template = repository_path(root, build["template"])
    tree = ET.parse(template)
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
        return os.path.relpath(root / path, project_dir).replace("/", "\\")

    # 模板路径相对于模板所在目录；新目标工程可以放在另一目录。
    for node in target.findall(".//FilePath"):
        source = (template.parent / (node.text or "").replace("\\", "/")).resolve()
        node.text = relative(source.relative_to(root.resolve()).as_posix())
    for node in target.findall(".//IncludePath"):
        paths = [value for value in (node.text or "").split(";") if value]
        node.text = ";".join(
            relative(
                (template.parent / value.replace("\\", "/"))
                .resolve()
                .relative_to(root.resolve())
                .as_posix()
            )
            for value in paths
        )
    targets = tree.getroot().find("Targets")
    for other in list(targets)[1:]:
        targets.remove(other)
    name = build["app_name"]
    set_value("TargetName", name)
    common = "TargetOption/TargetCommonOption/"
    output = relative(build["app_output"]) + "\\"
    app = int(config["TARGET_APP_BASE"].rstrip("U"), 0)
    size = int(config["TARGET_APP_BYTES"].rstrip("U"), 0)
    ram = int(config["TARGET_RAM_BASE"].rstrip("U"), 0)
    end = int(config["TARGET_RAM_END"].rstrip("U"), 0)
    set_value(common + "Device", profile["device"])
    set_value(common + "OutputName", name)
    set_value(common + "OutputDirectory", output)
    set_value(common + "ListingPath", output)
    set_value(
        common + "Cpu",
        f"IRAM(0x{ram:X}-0x{end - 1:X}) IROM(0x{app:X}-0x{app + size - 1:X}) " + build["cpu"],
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
        + '" "#L" --target '
        + profile["target"]
    )
    ads = "TargetOption/TargetArmAds/"
    defines = target.find(ads + "Cads/VariousControls/Define")
    # 配套APP版本来自共同目标清单，禁止运行时与描述符独立赋值。
    defines.text = build.get("app_defines", build["defines"]) + (
        ",YG_APP_LOADER_IMAGE=1,YG_APP_IMAGE_VERSION=TARGET_APP_VERSION,YG_APP_IMAGE_ROLE=1U"
    )
    set_value(ads + "LDads/umfTarg", "0")
    set_value(ads + "LDads/useFile", "1")
    set_value(ads + "LDads/ScatterFile", relative(build["target_directory"] + "/app.sct"))
    for item in target.find(ads + "ArmAdsMisc/OnChipMemories"):
        start, length = item.find("StartAddress"), item.find("Size")
        if start is not None and length is not None:
            if item.tag in ("IROM", "OCR_RVCT4"):
                start.text, length.text = hex(app), hex(size)
            elif item.tag in ("IRAM", "OCR_RVCT9"):
                start.text, length.text = hex(ram), hex(end - ram)
            else:
                start.text, length.text = "0x0", "0x0"
    includes = target.find(ads + "Cads/VariousControls/IncludePath")
    includes.text = (includes.text or "") + ";" + relative(build["selection_directory"])
    groups = target.find("Groups")
    for file in groups.findall("Group/Files/File"):
        if file.findtext("FileName") == "boot_hw_unavailable.c":
            file.find("FileName").text = Path(build["app_boot_source"]).name
            file.find("FilePath").text = relative(build["app_boot_source"])
    files = groups.find("Group[GroupName='50_Shared']/Files")
    if files is None:
        raise ValueError("普通APP工程缺少50_Shared分组")
    # APP 只保留进 Loader 的 RAM 邮箱与内嵌描述符；镜像校验与启动判据归 Loader。
    for source in (
        "shared/boot/image_mailbox.c",
        build["descriptor_source"],
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
    return ET.tostring(tree.getroot(), encoding="utf-8", xml_declaration=True).decode("utf-8")


def main() -> None:
    """生成配套 APP 工程，不构建、不访问设备。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="目标名称或目标 JSON；省略保持 G431")
    args = parser.parse_args()
    profile = load_target(args.target)
    content = render_project(profile, ROOT)
    project = repository_path(ROOT, profile["build"]["app_project"])
    project.parent.mkdir(parents=True, exist_ok=True)
    project.write_text(content, encoding="utf-8")


if __name__ == "__main__":
    main()
