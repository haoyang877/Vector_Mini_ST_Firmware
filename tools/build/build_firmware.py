"""重建旧板APP或AT32独立目标；只构建，不下载或连接探针。"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import KEIL, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uv4", default=os.environ.get("KEIL_UV4") or shutil.which("UV4.exe"))
    parser.add_argument(
        "--target",
        choices=(
            "normal",
            "all",
            "r40-foundation",
            "r40-openloop-dry",
            "r40-openloop-power",
            "r40-openloop-combined",
            "r40-closedloop",
            "r40-app-off",
        ),
        default="all",
    )
    args = parser.parse_args()
    if not args.uv4 or not Path(args.uv4).is_file():
        parser.error("Keil executable not found; set KEIL_UV4 or specify --uv4 /path/to/UV4.exe")
    log_dir = ROOT / "outputs/build/logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    targets = {
        "normal": KEIL / "YG_ESC_APP.uvprojx",
        "r40-foundation": ROOT
        / "firmware/platform/at32f456/foundation/R40_F456_Foundation.uvprojx",
        "r40-openloop-dry": ROOT
        / "firmware/platform/at32f456/foundation/R40_F456_OpenLoop_Dry.uvprojx",
        "r40-openloop-power": ROOT
        / "firmware/platform/at32f456/foundation/R40_F456_OpenLoop_Power.uvprojx",
        "r40-openloop-combined": ROOT
        / "firmware/platform/at32f456/foundation/R40_F456_OpenLoop_Combined.uvprojx",
    }
    targets["r40-closedloop"] = (
        ROOT / "firmware/platform/at32f456/foundation/R40_F456_ClosedLoop.uvprojx"
    )
    targets["r40-app-off"] = (
        ROOT / "firmware/platform/at32f456/application/R40_F456_App_Off.uvprojx"
    )
    for kind, project in targets.items():
        # all保留既有含义，底层验证镜像只接受显式选择，不能混入APP发布。
        if args.target != kind and not (args.target == "all" and kind == "normal"):
            continue
        selection = project.stem
        is_openloop = kind.startswith("r40-openloop-")
        log = log_dir / ((selection if is_openloop else project.stem) + ".log")
        command = [args.uv4, "-r", str(project), "-j0", "-o", str(log)]
        if is_openloop:
            command += ["-t", selection]
        # Keil 的 -j0 隐藏 GUI 并抑制交互消息；不是编译任务并行度选项。
        result = subprocess.run(command, cwd=project.parent)
        text = log.read_text(errors="replace") if log.exists() else ""
        print("\n".join(text.splitlines()[-6:]))
        if result.returncode > 1 or "0 Error(s)" not in text or "0 Warning(s)" not in text:
            print(f"Build failed. See {log}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
