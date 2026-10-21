"""显式单板CAN升级耐久测试；每圈核对参数、版本、UID和功率安全。"""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import sys
import time
import zlib

from loader_updater import (
    Client,
    LoaderBus,
    app_parameter_crc,
    app_query,
    app_ready,
    validate_image,
)


def validate_power_samples(first, second):
    """只比较功率相关位；SPI/其他GPIO活动不属于六路关相判据。"""
    masks = (0x555, 0x3FC000, 15)
    a = tuple(value & mask for value, mask in zip(first, masks))
    b = tuple(value & mask for value, mask in zip(second, masks))
    if a != b or a != (0, 0x3FC000, 15):
        raise RuntimeError(
            f"Power disable not verified: raw={first}/{second}, masked={a}/{b}"
        )


def verify_outputs(args, uid):
    """只读指定探针，禁止通过探针枚举顺序选择板；失败保持暂停并中止。"""
    import pylink  # 动态依赖只用于本显式硬件测试入口。

    j = pylink.JLink(lib=pylink.library.Library(dllpath=str(args.jlink_dll)))
    opened = False
    connected = False
    matched = False
    was_halted = None
    safe = False
    failure = None
    try:
        j.open(serial_no=args.jlink_serial)
        opened = True
        j.exec_command("SetRestartOnClose = 0")
        j.set_tif(pylink.enums.JLinkInterfaces.SWD)
        j.connect("STM32G431CB", speed=100)
        connected = True
        was_halted = j.halted()
        actual_uid = struct.pack("<3I", *j.memory_read32(0x1FFF7590, 3))
        if actual_uid != uid:
            raise RuntimeError("J-Link UID changed")
        matched = True
        if was_halted:
            raise RuntimeError("Target halted unexpectedly")

        def sample():
            return tuple(
                j.memory_read32(a, 1)[0] for a in (0x40012C20, 0x48000000, 0x48000400)
            )

        first, second = sample(), sample()
        validate_power_samples(first, second)
        safe = True
    except BaseException as exc:
        failure = exc
        raise
    finally:
        cleanup_errors = []
        if connected and matched and not safe:
            # 在已绑定板上硬件关相并暂停；仍保留失败并要求现场介入。
            try:
                j.halt()
                for address, clear, set_bits in (
                    (0x40012C20, 0x555, 0),
                    (0x40012C44, 0x8000, 0),
                    (0x48000000, 0x3FC000, 0x3FC000),
                    (0x48000400, 15, 15),
                ):
                    value = j.memory_read32(address, 1)[0]
                    j.memory_write32(address, [(value & ~clear) | set_bits])
            except Exception as exc:
                cleanup_errors.append(f"emergency disable: {exc}")
        restart = safe or (connected and not matched and was_halted is False)
        if opened:
            try:
                j.exec_command("SetRestartOnClose = " + ("1" if restart else "0"))
            except Exception as exc:
                cleanup_errors.append(f"restart policy: {exc}")
            try:
                j.close()
            except Exception as exc:
                cleanup_errors.append(f"probe close: {exc}")
        if cleanup_errors:
            raise RuntimeError(
                f"J-Link verification failed: {failure}; cleanup: {'; '.join(cleanup_errors)}"
            ) from failure


def main():
    """一批固定镜像连续测试；任一失败即停止，不重置失败计数继续凑1000次。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--dll", type=Path, required=True)
    p.add_argument("--adapter-serial", required=True)
    p.add_argument("--device-index", type=int, default=0)
    p.add_argument("--channel", type=int, choices=(0, 1), required=True)
    p.add_argument("--node", type=int, required=True)
    p.add_argument("--uid", required=True)
    p.add_argument("--jlink-serial", type=int, required=True)
    p.add_argument("--jlink-dll", type=Path, required=True)
    p.add_argument("--pylink-dir", type=Path)
    p.add_argument("--parameter-backup", type=Path, required=True)
    p.add_argument("--package", type=Path, required=True)
    p.add_argument("--cycles", type=int, default=1000)
    p.add_argument("--out", type=Path, required=True)
    args = p.parse_args()
    if args.pylink_dir:
        sys.path.insert(0, str(args.pylink_dir))
    uid = bytes.fromhex(args.uid)
    if len(uid) != 12 or not 0 <= args.node <= 7 or not 1 <= args.cycles <= 1000:
        raise ValueError("Target/count invalid")
    image = (args.package / "app.bin").read_bytes()
    manifest = json.loads((args.package / "manifest.json").read_text(encoding="utf-8"))
    validate_image(image)
    if (
        hashlib.sha256(image).hexdigest() != manifest["image_sha256"]
        or zlib.crc32(image) & 0xFFFFFFFF != manifest["image_crc32"]
    ):
        raise RuntimeError("Package changed")
    baseline = args.parameter_backup.read_bytes()
    if len(baseline) == 0x20000:
        baseline = baseline[0x1C000:]
    if len(baseline) != 0x4000:
        raise ValueError("Need complete 16 KiB parameters or full Flash backup")
    expected_crc = zlib.crc32(baseline) & 0xFFFFFFFF
    args.out.mkdir(parents=True, exist_ok=True)
    if (args.out / "summary.json").exists() or (args.out / "cycles.csv").exists():
        raise RuntimeError("Output batch already exists; preserve previous evidence")
    bus = LoaderBus(args.dll, (args.channel,), args.device_index, args.adapter_serial)
    report = {
        "status": "RUNNING",
        "requested_cycles": args.cycles,
        "attempted": 0,
        "passed": 0,
        "failed": 0,
        "uid": args.uid,
        "node": args.node,
        "adapter": bus.info,
        "channel": args.channel,
        "jlink_serial": args.jlink_serial,
        "image_sha256": manifest["image_sha256"],
        "app_version": manifest["app_version"],
        "parameter_sha256": hashlib.sha256(baseline).hexdigest(),
        "parameter_crc32": expected_crc,
        "elapsed_seconds": 0.0,
    }
    started = time.monotonic()

    def save():
        report["elapsed_seconds"] = round(time.monotonic() - started, 3)
        report["observed_failure_rate"] = report["failed"] / max(report["attempted"], 1)
        temp = args.out / "summary.tmp"
        temp.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        temp.replace(args.out / "summary.json")

    save()
    try:
        app_ready(bus, args.channel, args.node, manifest["app_version"], uid)
        if app_parameter_crc(bus, args.channel, args.node) != expected_crc:
            raise RuntimeError("Initial parameters differ from backup")
        verify_outputs(args, uid)
        with (args.out / "cycles.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=(
                    "cycle",
                    "result",
                    "duration_s",
                    "parameter_crc",
                    "requests",
                    "retransmissions",
                    "error",
                ),
            )
            writer.writeheader()
            for cycle in range(1, args.cycles + 1):
                if (args.out / "stop.request").exists():
                    report["status"] = "STOPPED"
                    break
                cycle_start = time.monotonic()
                row = {
                    "cycle": cycle,
                    "result": "FAIL",
                    "duration_s": 0,
                    "parameter_crc": None,
                    "requests": 0,
                    "retransmissions": 0,
                    "error": "",
                }
                report["attempted"] += 1
                client = Client(bus, args.channel, args.node)
                try:
                    app_query(
                        bus,
                        args.channel,
                        args.node,
                        1,
                        b"\0" * 4,
                        lambda data: data == b"\0" * 4,
                    )
                    if app_parameter_crc(bus, args.channel, args.node) != expected_crc:
                        raise RuntimeError("Parameters changed before cycle")
                    app_query(
                        bus,
                        args.channel,
                        args.node,
                        0x66,
                        b"BOOT",
                        lambda data: data == b"\0" * 4,
                    )
                    time.sleep(0.08)
                    client.update(image, manifest["app_version"], uid)
                    if client.parameter_crc != expected_crc:
                        raise RuntimeError("Loader parameter CRC changed")
                    app_ready(
                        bus, args.channel, args.node, manifest["app_version"], uid
                    )
                    row["parameter_crc"] = app_parameter_crc(
                        bus, args.channel, args.node
                    )
                    if row["parameter_crc"] != expected_crc:
                        raise RuntimeError("Parameters changed after cycle")
                    app_query(
                        bus,
                        args.channel,
                        args.node,
                        1,
                        b"\0" * 4,
                        lambda data: data == b"\0" * 4,
                    )
                    verify_outputs(args, uid)
                    row["result"] = "PASS"
                    report["passed"] += 1
                except Exception as exc:
                    row["error"] = str(exc)
                    report["failed"] += 1
                    report["status"] = "FAIL"
                    report["last_error"] = str(exc)
                    diagnostics = {
                        "cycle": cycle,
                        "error": str(exc),
                        "last_requests": client.trace[-40:],
                    }
                    try:
                        diagnostics["adapter_status"] = bus.diagnostics(args.channel)
                    except Exception as diagnostic_error:
                        diagnostics["adapter_status_error"] = str(diagnostic_error)
                    (args.out / f"failure_{cycle:04d}.json").write_text(
                        json.dumps(diagnostics, indent=2) + "\n", encoding="utf-8"
                    )
                row["duration_s"] = round(time.monotonic() - cycle_start, 3)
                row["requests"] = len(client.trace)
                row["retransmissions"] = sum(x["attempt"] > 0 for x in client.trace)
                writer.writerow(row)
                f.flush()
                save()
                if cycle == 1 or cycle % 10 == 0 or row["result"] != "PASS":
                    print(
                        json.dumps(
                            {
                                "cycle": cycle,
                                "result": row["result"],
                                "duration_s": row["duration_s"],
                                "passed": report["passed"],
                                "failed": report["failed"],
                                "error": row["error"],
                            }
                        ),
                        flush=True,
                    )
                if row["result"] != "PASS":
                    break
            if report["passed"] == args.cycles and report["failed"] == 0:
                report["status"] = "PASS"
    except Exception as exc:
        report["status"] = "FAIL"
        report["last_error"] = str(exc)
        raise
    finally:
        save()
        bus.close()
    print(json.dumps(report, indent=2), flush=True)
    if report["status"] != "PASS":
        raise RuntimeError("Qualification incomplete/failed; no automatic retry")


if __name__ == "__main__":
    main()
