"""独立构建位置参考候选，不下载、不打开调试器、不接触 CAN。"""

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from project_paths import KEIL, ROOT


def main() -> int:
    """生成隔离 Keil 工程并完整重编译所有相关编译单元。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--uv4", default=shutil.which("UV4.exe"))
    parser.add_argument("--reference", choices=("point-to-point", "external-50hz", "external-200hz"), default="point-to-point")
    parser.add_argument("--can", choices=("classic", "fd-compatible"), default="classic")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    if not args.uv4 or not Path(args.uv4).is_file():
        parser.error("请使用 --uv4 指定 UV4.exe")
    out = (args.out or ROOT / "outputs/build/candidates" / f"{args.reference}-{args.can}").resolve()
    if not out.is_relative_to(ROOT / "outputs"):
        parser.error("构建产物必须位于本仓库 outputs/ 内")
    out.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(KEIL / "Vector_Mini_ST.uvprojx")
    definitions = {
        "POSITION_EXTERNAL_REFERENCE_ENABLE": int(args.reference.startswith("external-")),
        "VECTOR_CAN_CLASSIC_ONLY": int(args.can == "classic"),
    }
    for element in tree.iter():
        if element.tag in ("FilePath", "ScatterFile") and element.text:
            element.text = str((KEIL / element.text.replace("\\", "/")).resolve())
        if element.tag == "IncludePath" and element.text:
            element.text = ";".join(
                value if value.startswith("$") else str((KEIL / value.replace("\\", "/")).resolve())
                for value in element.text.split(";") if value
            )
        if element.tag in ("OutputDirectory", "ListingPath"):
            element.text = str(out) + "\\"
        if element.tag in ("RunUserProg1", "RunUserProg2"):
            element.text = "0"
    define = tree.find(".//TargetArmAds/Cads/VariousControls/Define")
    if define is None:
        raise RuntimeError("Keil C 全局定义缺失")
    define.text = (define.text or "") + "," + ",".join(f"{k}={v}" for k, v in definitions.items())
    project = out / "position_candidate.uvprojx"
    tree.write(project, encoding="utf-8", xml_declaration=True)
    log = out / "build.log"
    result = subprocess.run([args.uv4, "-r", str(project), "-j0", "-o", str(log)], cwd=out)
    text = log.read_text(errors="replace") if log.exists() else ""
    print("\n".join(text.splitlines()[-6:]))
    if result.returncode > 1 or "0 Error(s)" not in text:
        return 1
    axf = out / "Vector_Mini_ST.axf"
    subprocess.run([sys.executable, str(ROOT / "tools/build/keil_app_record.py"), str(axf)], check=True)
    tracked = ["firmware/app/foc_run.c", "firmware/motor/position/position_cascade.c",
               "firmware/motor/position/position_cascade_config.h", "firmware/communication/can/interface_can.c",
               "firmware/app/firmware_version.h"]
    manifest = {
        "status": "PASS", "hardware_accessed": False,
        "definitions": definitions, "reference": args.reference, "can": args.can,
        "source_sha256": {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in tracked},
        "artifact_sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in out.iterdir() if p.suffix in (".axf", ".bin", ".hex")},
    }
    (out / "summary.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
