"""G431生产身份工装：离线生成独立页；SWD写入须显式独占、无动力并核对UID和数字关相。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import struct
import subprocess
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT

MAGIC = 0x59475331


def encode_record(sn: str, motor_type: int) -> bytes:
    """校验生产字段并生成64B记录，末双字为CRC32与提交识别字。"""
    if re.fullmatch(r"[A-Za-z0-9-]{1,16}", sn) is None or not 1 <= motor_type <= 0xFFFFFFFF:
        raise ValueError("SN须为1..16位ASCII字母/数字/连字符，motor-type须为非零u32")
    prefix = struct.pack("<II16sI28s", MAGIC, 1, sn.encode("ascii"), motor_type, bytes(28))
    return prefix + struct.pack("<II", zlib.crc32(prefix), (~MAGIC) & 0xFFFFFFFF)


def run_jlink(executable: Path, probe: str, commands: list[str], output: Path, name: str) -> None:
    """执行显式SWD命令文件并保存原始日志；连接/退出异常不能继续生产写入。"""
    script = output / f"{name}.jlink"
    script.write_text("\n".join(commands + ["exit"]) + "\n", encoding="ascii")
    result = subprocess.run(
        [
            str(executable),
            "-Device",
            "STM32G431CB",
            "-If",
            "SWD",
            "-Speed",
            "1000",
            "-SelectEmuBySN",
            probe,
            "-AutoConnect",
            "1",
            "-ExitOnError",
            "1",
            "-CommandFile",
            str(script),
        ],
        capture_output=True,
        timeout=60,
        check=False,
    )
    (output / f"{name}.log").write_bytes(result.stdout + result.stderr)
    if result.returncode != 0:
        raise RuntimeError(f"J-Link {name}失败；检查日志，不继续写入或自动恢复")


def provision(args: argparse.Namespace, page: bytes, output: Path) -> dict:
    """备份全部Flash后只写身份页；其他任一字节改变均为硬失败，目标保持halt。"""
    if not args.exclusive_bench or not args.no_motor_power or not args.uid or not args.probe:
        raise ValueError("SWD生产写须提供独占/无动力、实际UID和明确探针，不接受默认选择")
    expected_uid = bytes.fromhex(args.uid)
    if len(expected_uid) != 12:
        raise ValueError("UID须为12B十六进制")
    before, uid = output / "before.bin", output / "uid.bin"
    phase, gpio_a, gpio_b = output / "phase.bin", output / "gpio_a.bin", output / "gpio_b.bin"
    run_jlink(
        args.jlink,
        args.probe,
        [
            "halt",
            f'savebin "{before.as_posix()}", 0x08000000, 0x20000',
            f'savebin "{uid.as_posix()}", 0x1FFF7590, 12',
            f'savebin "{phase.as_posix()}", 0x40012C20, 4',
            f'savebin "{gpio_a.as_posix()}", 0x48000010, 8',
            f'savebin "{gpio_b.as_posix()}", 0x48000410, 8',
        ],
        output,
        "backup",
    )
    original = before.read_bytes()
    if len(original) != 0x20000 or uid.read_bytes() != expected_uid:
        raise RuntimeError("备份长度或UID不匹配，禁止生产写入")
    if (
        len(phase.read_bytes()) != 4
        or len(gpio_a.read_bytes()) != 8
        or len(gpio_b.read_bytes()) != 8
    ):
        raise RuntimeError("无法读取关相寄存器，禁止生产写入，须人工检查")
    ccer = struct.unpack("<I", phase.read_bytes())[0]
    a_in, a_out = struct.unpack("<II", gpio_a.read_bytes())
    b_in, b_out = struct.unpack("<II", gpio_b.read_bytes())
    if ccer & 0x555 or (a_in | a_out) & 0x780 or (b_in | b_out) & 3:
        raise RuntimeError("STOP/数字关相未确认，硬失败，须人工检查，禁止继续")
    if original[0x1F800:] != b"\xff" * 2048 and not args.replace_existing:
        raise ValueError("身份页非空，覆盖须显式--replace-existing；未执行写入")
    page_file, after = output / "identity_page.bin", output / "after.bin"
    run_jlink(
        args.jlink,
        args.probe,
        [
            "halt",
            f'loadbin "{page_file.as_posix()}", 0x0801F800',
            f'savebin "{after.as_posix()}", 0x08000000, 0x20000',
        ],
        output,
        "provision",
    )
    expected = original[:0x1F800] + page
    if after.read_bytes() != expected:
        raise RuntimeError("读回或保护区比较失败，目标保持halt；人工恢复备份，禁止继续")
    return {
        "programmed": True,
        "uid": expected_uid.hex(),
        "before_sha256": hashlib.sha256(original).hexdigest(),
        "after_sha256": hashlib.sha256(expected).hexdigest(),
        "target_halted": True,
    }


def main() -> int:
    """默认只生成文件；--jlink启用显式生产SWD写入，不开放CAN生产写权限。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sn", required=True)
    parser.add_argument("--motor-type", type=lambda value: int(value, 0), required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--jlink", type=Path)
    parser.add_argument("--probe")
    parser.add_argument("--uid")
    parser.add_argument("--replace-existing", action="store_true")
    parser.add_argument("--no-motor-power", action="store_true")
    parser.add_argument("--exclusive-bench", action="store_true")
    args = parser.parse_args()
    record = encode_record(args.sn, args.motor_type)
    page = record + b"\xff" * (2048 - len(record))
    output = args.output.resolve()
    if not output.is_relative_to((ROOT / "outputs").resolve()):
        parser.error("生产文件和证据必须在当前工作树outputs下")
    output.mkdir(parents=True, exist_ok=False)
    (output / "identity_record.bin").write_bytes(record)
    (output / "identity_page.bin").write_bytes(page)
    report = {
        "success": False,
        "programmed": False,
        "sn": args.sn,
        "motor_type": args.motor_type,
        "page_address": "0x0801F800",
        "record_sha256": hashlib.sha256(record).hexdigest(),
        "format": 1,
    }
    try:
        if args.jlink is not None:
            report.update(provision(args, page, output))
        report["success"] = True
    except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as exc:
        report["failure"] = str(exc)
    finally:
        (output / "summary.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
        )
    return 0 if report["success"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
