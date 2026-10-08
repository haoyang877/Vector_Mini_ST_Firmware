"""验证 CAN FD 只读链路端点的打包、路由和响应闭环；不访问硬件。"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def check_lifecycle_vectors() -> None:
    path = ROOT / "docs/protocols/yg_protocol_motor_lifecycle_wire_v1_4_vectors.json"
    vectors = json.loads(path.read_text(encoding="utf-8"))["vectors"]
    if len(vectors) != 4:
        raise ValueError("生命周期线路须有两个完整请求和两个完整回复向量")
    for vector in vectors:
        data = bytes.fromhex(vector["data_hex"])
        if len(data) != vector["data_length"] or {24: 12, 48: 14}.get(len(data)) != vector["dlc"]:
            raise ValueError(f"{vector['name']}: CAN FD DLC 与数据长度不符")
        can_id = int(vector["can_id"], 16)
        if (
            data[:2] != b"\x5a\xa5"
            or data[4] != (can_id & 0xFF)
            or data[5] != ((can_id >> 8) & 0xFF)
        ):
            raise ValueError(f"{vector['name']}: 应用头与 CAN ID 不符")
        payload_length = int.from_bytes(data[10:14], "little")
        end = 16 + payload_length
        if end + 2 > len(data) or any(data[end + 2 :]):
            raise ValueError(f"{vector['name']}: 应用长度或 DLC 补零错误")
        crc8 = 0
        for byte in data[:15]:
            crc8 ^= byte
            for _ in range(8):
                crc8 = ((crc8 << 1) ^ (0x9B if crc8 & 0x80 else 0)) & 0xFF
        crc16 = 0xFFFF
        for byte in data[:end]:
            crc16 ^= byte << 8
            for _ in range(8):
                crc16 = ((crc16 << 1) ^ (0xBAAD if crc16 & 0x8000 else 0)) & 0xFFFF
        if data[15] != crc8 or int.from_bytes(data[end : end + 2], "little") != crc16:
            raise ValueError(f"{vector['name']}: 应用 CRC 错误")


def main() -> None:
    check_lifecycle_vectors()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig 或 C 编译器，请通过 --cc 指定")
    output_dir = (args.out or ROOT / "outputs/tests/yg_protocol_link").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "yg_protocol_link.exe" if __import__("os").name == "nt" else "yg_protocol_link"
    )
    command = [str(compiler)]
    if compiler.stem.lower() == "zig":
        command.append("cc")
    command.extend(
        NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(ROOT / "tests/unit/yg_protocol_link_test.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_link.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_runtime.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_canfd.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_endpoint.c"),
            str(ROOT / "shared/protocol/yg_protocol_crc.c"),
            str(ROOT / "shared/protocol/yg_protocol_frame_codec.c"),
            str(ROOT / "shared/protocol/yg_protocol_can_id.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_message_registry.c"),
            str(ROOT / "shared/protocol/yg_protocol_fragment.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_router.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_parameter.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_job.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_boot.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor_status.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_readonly.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_readonly_payload.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_transfer.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
