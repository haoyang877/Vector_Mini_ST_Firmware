"""编译原APP源码及虚拟端口，执行静态启动或最小速度闭环，不访问硬件。"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """从真实normal工程选择可移植生产源码，拒绝隐式链接STM32实现。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, help="原生C编译器或zig可执行文件")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--scenario", choices=("startup", "speed"), default="startup")
    parser.add_argument("--substeps", type=int, choices=range(1, 101), default=10)
    args = parser.parse_args()
    output = (
        args.out or ROOT / "outputs/virtual_bench" / f"{args.scenario}-{args.substeps}"
    ).resolve()
    output.mkdir(parents=True, exist_ok=True)
    project = ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx"
    target = ET.parse(project).getroot().find("./Targets/Target")
    if target is None:
        raise SystemExit("normal工程缺少Target")
    # 只继承选型宏；不把HAL/MCU或Loader构建宏引入可移植代码。
    defines = target.findtext("./TargetOption/TargetArmAds/Cads/VariousControls/Define", "")
    selected_profiles = [
        value.strip()
        for value in defines.split(",")
        if value.strip().split("=", 1)[0]
        in ("CURRENT_SENSE_SHUNT_MILLIOHM", "FOC_ACTIVE_HW_PROFILE", "FOC_ACTIVE_MOTOR_PROFILE")
    ]
    sources = []
    prefixes = (
        "firmware/app/",
        "firmware/motor/",
        "firmware/parameters/",
        "firmware/communication/",
        "shared/protocol/",
        "shared/checksum/",
        "shared/boot/",
    )
    for element in target.findall("./Groups/Group/Files/File/FilePath"):
        source = (project.parent / element.text.replace("\\", "/")).resolve()
        relative = source.relative_to(ROOT).as_posix()
        if source.suffix == ".c" and relative.startswith(prefixes):
            sources.append(source)
    if not sources or len(sources) != len(set(sources)):
        raise SystemExit("生产源码清单为空或包含重复项")
    sources += [
        ROOT / "firmware/platform/stm32g4/ports/boot/boot_hw_unavailable.c",
        ROOT / "firmware/platform/virtual/virtual_platform.c",
        ROOT / "firmware/platform/virtual/inverter_model.c",
        ROOT / "firmware/platform/virtual/motor_model.c",
        ROOT / "tests/virtual_bench" / ("main.c" if args.scenario == "startup" else "speed.c"),
    ]
    executable = output / "virtual_bench.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [f"-D{value}" for value in selected_profiles]
        + [
            "-I",
            str(ROOT / "firmware/platform/virtual"),
            "-std=c99",
            "-O1",
            "-Wall",
            "-Wextra",
            "-Werror",
            *map(str, sources),
            "-lm",
            "-o",
            str(executable),
        ]
    )
    inputs = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sources
    }
    for path in (project, Path(__file__).resolve(), ROOT / "tools/project_paths.py"):
        inputs[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    # 头文件也属于输入；完整记录避免只看HEAD而忽略工作区变化。
    for directory in (ROOT / "firmware", ROOT / "shared"):
        for path in directory.rglob("*.h"):
            if "cubemx" not in path.parts and "third_party" not in path.parts:
                inputs[str(path.relative_to(ROOT))] = hashlib.sha256(path.read_bytes()).hexdigest()
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=False)
    (output / "compile.log").write_text(result.stdout + result.stderr, encoding="utf-8")
    summary = {
        "success": False,
        "scope": args.scenario,
        "integration_substeps": args.substeps,
        "project_profile_defines": selected_profiles,
        "hardware_access": False,
        "inputs": inputs,
        "command": command,
        "compile_exit": result.returncode,
        "source_head": subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip(),
    }
    if result.returncode == 0:
        run = subprocess.run(
            [str(executable), str(args.substeps)],
            cwd=output,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
        (output / "run.log").write_text(run.stdout + run.stderr, encoding="utf-8")
        summary["run_exit"] = run.returncode
        summary["success"] = run.returncode == 0
        summary["results"] = [
            json.loads(line) for line in run.stdout.splitlines() if line.startswith("{")
        ]
        summary["artifacts"] = {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (executable, output / "speed.csv", output / "parameter_seed.bin")
            if path.is_file()
        }
        print(run.stdout, end="")
        print(run.stderr, end="", file=sys.stderr)
    else:
        print(result.stdout + result.stderr)
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    if not summary["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
