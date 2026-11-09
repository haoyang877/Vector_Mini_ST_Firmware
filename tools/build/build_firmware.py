"""重建旧板APP或AT32独立目标；只构建，不下载或连接探针。"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import KEIL, ROOT

# 发布验证覆盖两板普通APP及配套Loader产品；台架镜像只接受显式选择。
RELEASE_PROJECTS = {
    "normal": KEIL / "ESC-APP-STM32G431.uvprojx",
    "at32f456-app-off": ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx",
    "g431-loader": ROOT / "loader/targets/vector_mini_st_g431/YG2026_ESC_LOADER.uvprojx",
    "g431-loader-app": KEIL / "ESC-APP-STM32G431-LoaderApp.uvprojx",
    "at32-loader": ROOT / "loader/targets/yg_esc_v1_at32f456/ESC-LOADER-AT32F456.uvprojx",
    "at32-loader-app": ROOT / "projects/at32f456/application/ESC-APP-AT32F456-LoaderApp.uvprojx",
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uv4", default=os.environ.get("KEIL_UV4") or shutil.which("UV4.exe"))
    parser.add_argument(
        "--target",
        choices=(
            "normal",
            "all",
            "at32f456-foundation",
            "at32f456-openloop-dry",
            "at32f456-openloop-power",
            "at32f456-openloop-combined",
            "at32f456-closedloop",
            "at32f456-app-off",
            "g431-loader",
            "g431-loader-app",
            "at32-loader",
            "at32-loader-app",
        ),
        default="all",
    )
    args = parser.parse_args()
    if not args.uv4 or not Path(args.uv4).is_file():
        parser.error("Keil executable not found; set KEIL_UV4 or specify --uv4 /path/to/UV4.exe")
    log_dir = ROOT / "outputs/build/logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    targets = {
        "normal": KEIL / "ESC-APP-STM32G431.uvprojx",
        "at32f456-foundation": ROOT / "projects/at32f456/bench/AT32F456_Foundation.uvprojx",
        "at32f456-openloop-dry": ROOT / "projects/at32f456/bench/AT32F456_OpenLoop_Dry.uvprojx",
        "at32f456-openloop-power": ROOT / "projects/at32f456/bench/AT32F456_OpenLoop_Power.uvprojx",
        "at32f456-openloop-combined": ROOT
        / "projects/at32f456/bench/AT32F456_OpenLoop_Combined.uvprojx",
    }
    targets["at32f456-closedloop"] = ROOT / "projects/at32f456/bench/AT32F456_ClosedLoop.uvprojx"
    targets["at32f456-app-off"] = ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx"
    targets.update(RELEASE_PROJECTS)
    for kind, project in targets.items():
        # 默认构建完整产品集合，不把底层台架镜像混入发布。
        if args.target != kind and not (args.target == "all" and kind in RELEASE_PROJECTS):
            continue
        selection = project.stem
        is_openloop = kind.startswith("at32f456-openloop-")
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
