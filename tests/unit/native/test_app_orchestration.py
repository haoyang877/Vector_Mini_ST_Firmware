"""用完整APP源码验证健康监督、Loader交接和身份发布，可与重构前源码逐事件对照。"""

import argparse
import subprocess
import sys
from pathlib import Path

from _bsp_test_support import run_c
from run_position_servo_tests import ROOT


def main() -> None:
    """按两板和有无Loader编译，选配同输入下重构前后的逐事件摘要对比。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--reference-source", type=Path, help="含原生测试的基线源码树中的 app.c")
    args = parser.parse_args()
    args.out = args.out.resolve()
    args.out.mkdir(parents=True, exist_ok=True)
    fixture = Path(__file__).with_name("app_orchestration_fixture.c").read_text(encoding="utf-8")
    current = (ROOT / "firmware/app/app.c").read_text(encoding="utf-8")
    loader_source = (ROOT / "firmware/app/app_loader.c").read_text(encoding="utf-8")
    extra = [ROOT / "shared/boot/device_identity.c", ROOT / "shared/checksum/crc32.c"]
    if args.reference_source:
        tree = next(
            p for p in args.reference_source.resolve().parents if (p / "tests/run.py").exists()
        )
        before = args.out / "reference"
        subprocess.run(
            [
                sys.executable,
                str(tree / "tests/unit/native/test_app_orchestration.py"),
                "--cc",
                args.cc,
                "--out",
                str(before),
            ],
            cwd=tree,
            check=True,
        )
    for board in (1, 2):
        for loader in (False, True):
            name = f"app_{board}_{int(loader)}"
            prefix = f"#define YG_APP_LOADER_IMAGE {int(loader)}\n"
            if board == 2:
                prefix += "#define APP_TARGET_AT32_F456 1\n"
            run_c(args, name, prefix + current + loader_source + fixture, board, extra)
            if args.reference_source:
                outputs = [
                    subprocess.check_output([str(folder / (name + ".exe"))])
                    for folder in (args.out, before)
                ]
                if outputs[0] != outputs[1]:
                    raise RuntimeError(f"{name}重构前后的APP事件轨迹不一致")


if __name__ == "__main__":
    main()
