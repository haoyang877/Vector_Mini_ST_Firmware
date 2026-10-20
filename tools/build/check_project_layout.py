"""检查 Keil 工程的源文件路径、include 目录与工程内 include 解析。

只读文件，不构建、不烧录、不接触硬件；从任意目录运行：
python tools/build/check_project_layout.py
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import argparse
import re
import sys
import xml.etree.ElementTree as ET

from project_paths import ROOT

sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target, repository_path  # noqa: E402


def resolve_path(base, value):
    return (base / value.replace("\\", "/")).resolve()


def check_project(project):
    errors = []
    tree = ET.parse(project)
    include_dirs = []
    sources = []
    for node in tree.findall(".//IncludePath"):
        for value in (node.text or "").split(";"):
            if not value:
                continue
            path = resolve_path(project.parent, value)
            if re.match(r"^[A-Za-z]:", value) or value.startswith(("/", "\\")):
                errors.append(f"non-portable include directory: {value}")
            if not path.is_dir():
                errors.append(f"missing include directory: {value}")
            if path not in include_dirs:
                include_dirs.append(path)
    for node in tree.findall(".//FilePath"):
        value = node.text or ""
        path = resolve_path(project.parent, value)
        if re.match(r"^[A-Za-z]:", value) or value.startswith(("/", "\\")):
            errors.append(f"non-portable source path: {value}")
        if not path.is_file():
            errors.append(f"missing source: {value}")
        if path in sources:
            errors.append(f"duplicate source: {value}")
        sources.append(path)

    # 从实际编译单元追踪自有头文件，避免把其他目标的专用端口误套进本工程。
    owned = (
        "firmware/app",
        "firmware/motor",
        "firmware/services",
        "firmware/communication",
        "firmware/platform",
        "firmware/third_party/segger_rtt",
        "loader",
        "shared",
    )
    roots = [(ROOT / folder).resolve() for folder in owned]
    pending = list(sources)
    visited = set()
    while pending:
        source = pending.pop()
        if (
            source in visited
            or not source.is_file()
            or source.suffix not in (".c", ".h")
            or not any(source.is_relative_to(root) for root in roots)
            or "Drivers" in source.parts
            or source.name == "stm32g4xx_hal_conf.h"
        ):
            continue
        visited.add(source)
        text = source.read_text(encoding="utf-8-sig", errors="surrogateescape")
        for value in re.findall(r'^\s*#\s*include\s+"([^"]+)"', text, re.M):
            matches = [
                resolve_path(base, value)
                for base in [source.parent, *include_dirs]
                if resolve_path(base, value).is_file()
            ]
            if not matches:
                errors.append(f'{source.relative_to(ROOT)}: unresolved include "{value}"')
            else:
                pending.append(matches[0])
    return errors, len(sources), len(include_dirs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", help="目标名称或目标 JSON；省略保持 G431")
    args = parser.parse_args()
    profile = load_target(args.target)
    build = profile["build"]
    failed = False
    for relative in dict.fromkeys(
        (build["template"], build["loader_project"], build["app_project"])
    ):
        project = repository_path(ROOT, relative)
        name = project.stem
        errors, sources, includes = check_project(project)
        for error in errors:
            print(f"{name}: {error}", file=sys.stderr)
        print(f"{name}: {sources} files, {includes} include directories, {len(errors)} errors")
        failed |= bool(errors)
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
