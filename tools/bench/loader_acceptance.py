"""显式编排CAN升级及前后SWD只读保护检查；不重新实现线协议。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
from device_maintenance import verify_protected  # noqa: E402
from target_contract import load_target  # noqa: E402


def run(arguments: list[str]) -> None:
    """任何阶段失败终止当前批次，保留子进程原始日志。"""
    subprocess.run([sys.executable, *arguments], cwd=ROOT, check=True)


def main() -> None:
    """只有显式提供两条物理链路与独占条件才执行硬件升级。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", required=True)
    parser.add_argument("--package", type=Path, required=True)
    parser.add_argument("--sdk", type=Path, required=True)
    parser.add_argument("--device-index", type=int, default=0)
    parser.add_argument("--node", type=int, required=True)
    parser.add_argument("--uid", required=True)
    parser.add_argument("--probe", type=int, required=True)
    parser.add_argument("--jlink-dll", type=Path)
    parser.add_argument("--cycles", type=int, default=1)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--exclusive-no-power", action="store_true")
    args = parser.parse_args()
    if not args.exclusive_no_power or not 1 <= args.cycles <= 1000:
        parser.error("必须明确无动力独占条件，循环次数为1..1000")
    if not args.out.resolve().is_relative_to((ROOT / "outputs").resolve()):
        parser.error("验收证据必须位于当前工作树outputs")
    target = load_target(args.target)
    args.out.mkdir(parents=True, exist_ok=False)
    report = {"success": False, "target": args.target, "uid": args.uid, "cycles": []}
    snapshot_arguments = [
        "loader/tools/device_maintenance.py",
        "backup",
        "--target",
        args.target,
        "--probe",
        str(args.probe),
        "--uid",
        args.uid,
        "--exclusive-no-power",
    ]
    if args.jlink_dll:
        snapshot_arguments += ["--jlink-dll", str(args.jlink_dll.resolve())]
    try:
        for cycle in range(1, args.cycles + 1):
            directory = args.out / f"cycle-{cycle:04d}"
            run([*snapshot_arguments, "--out", str((directory / "before").resolve())])
            run(
                [
                    "host_app/upgrade_client.py",
                    "upgrade",
                    "--target",
                    args.target,
                    "--package",
                    str(args.package.resolve()),
                    "--sdk",
                    str(args.sdk.resolve()),
                    "--device-index",
                    str(args.device_index),
                    "--node",
                    str(args.node),
                    "--uid",
                    args.uid,
                    "--exclusive-bench",
                    "--output",
                    str((directory / "upgrade").resolve()),
                ]
            )
            run([*snapshot_arguments, "--out", str((directory / "after").resolve())])
            before = (directory / "before/snapshot/flash.bin").read_bytes()
            after = (directory / "after/snapshot/flash.bin").read_bytes()
            checked = verify_protected(before, after, target)
            report["cycles"].append({"cycle": cycle, **checked})
            (args.out / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        report["success"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        (args.out / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
