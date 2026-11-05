"""从固件五轴配置生成一条全新 schema14 记录；不连接设备或写入固件。"""

import argparse
import math
import subprocess
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", type=lambda s: int(s, 0), required=True, choices=range(3, 8))
    parser.add_argument("--travel", nargs=2, type=float, metavar=("MIN_RAD", "MAX_RAD"))
    parser.add_argument("--speed", type=float, required=True, help="电机轴 rad/s")
    parser.add_argument(
        "--thermal", type=float, nargs=7, help="持续A、峰值A、时间常数s、进入/退出/跳闸/清除比"
    )
    parser.add_argument(
        "--output-shaft", action="store_true", help="输入行程和速度按输出轴，按角色减速比转换"
    )
    parser.add_argument(
        "--confirm-model", action="store_true", help="明确确认本机型号；编码器标定仍为空"
    )
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.node >= 5 and args.travel is None:
        parser.error("关节必须明确给出电机轴实际行程 --travel")
    if args.node >= 6 and args.thermal is None:
        parser.error("YQ 热模型需提供已核对的 --thermal 七个参数")
    travel = args.travel or [0.0, 0.0]
    values = [*travel, args.speed, *(args.thermal or [])]
    if not all(math.isfinite(v) for v in values):
        parser.error("参数必须为有限值")
    if args.output.exists():
        parser.error("输出已存在")
    cc = zig_path()
    if cc is None:
        parser.error("请安装仓库锁定的 Zig 工具链")
    with tempfile.TemporaryDirectory() as directory:
        work = Path(directory)
        executable = work / "make_axis_record.exe"
        record = work / "record.bin"
        sources = [
            "tools/build/make_axis_record.c",
            "firmware/parameters/parameter_record.c",
            "firmware/parameters/motor_axis_profile.c",
            "shared/checksum/crc32.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/math/control_math.c",
            "firmware/motor/control/speed_feedforward.c",
            "firmware/parameters/parameter_values.c",
        ]
        subprocess.run(
            [
                str(cc),
                "cc",
                *NATIVE_INCLUDE_FLAGS,
                "-DBSP_BOARD=2",
                "-Os",
                *sources,
                "-o",
                str(executable),
            ],
            cwd=ROOT,
            check=True,
        )
        subprocess.run(
            [
                str(executable),
                str(args.node),
                *map(str, travel),
                str(args.speed),
                str(int(args.confirm_model)),
                str(record),
                str(int(args.output_shaft)),
                *map(str, args.thermal or []),
            ],
            check=True,
        )
        with args.output.open("xb") as dest:
            dest.write(record.read_bytes())


if __name__ == "__main__":
    main()
