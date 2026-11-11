"""五轴 AT32F456 共线只读压力测试，每轴七万次，首次应答期限 50 ms。"""

import argparse
import csv
import ctypes as C
import json
import math
import os
import struct
import sys
import time
from collections import Counter
from contextlib import ExitStack
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "host_app"))
from ports.controlcanfd import ControlCanFd  # noqa: E402
from upgrade_client import decode_reply, encode_frames  # noqa: E402

PER_NODE = 70000
KINDS = (1, 2, 3, 108)
TIMEOUT = 0.050


def _upper_pct(n: int, errors: int, alpha: float) -> float:
    """反解二项累计概率，返回单侧精确置信上限（百分数）。"""
    if n == 0 or n == errors:
        return 100.0
    if errors == 0:
        return -100 * math.expm1(math.log(alpha) / n)

    def log_cdf(p):
        logs = [n * math.log1p(-p)]
        step = math.log(p) - math.log1p(-p)
        for i in range(1, errors + 1):
            logs.append(logs[-1] + math.log((n + 1 - i) / i) + step)
        largest = max(logs)
        return largest + math.log(sum(math.exp(x - largest) for x in logs))

    lo, hi = 0.0, 1.0
    for _ in range(55):
        mid = (lo + hi) / 2
        if log_cdf(mid) > math.log(alpha):
            lo = mid
        else:
            hi = mid
    return 100 * hi


def _validate(uid: bytes, node: int, kind: int, payload: bytes) -> None:
    if kind == 1 and (
        len(payload) != 32
        or payload[:12] != uid
        or struct.unpack_from("<II", payload, 20) != (0x602, 0x20200)
    ):
        raise ValueError("UID/version")
    if kind == 2 and payload != struct.pack("<I6H", 0x7FD00, 0x64E, 64, 46, 0, 500, 500):
        raise ValueError("capabilities")
    if kind == 3 and payload != struct.pack("<HHBBHQ", 6, 0, 1, 0, 0, 0):
        raise ValueError("device status")
    if kind == 108:
        if (
            len(payload) != 45
            or struct.unpack_from("<HH", payload) != (0, 0)
            or payload[42] != node
        ):
            raise ValueError("motor feedback: " + payload.hex())


def _latency(values: list[float]) -> dict[str, float]:
    data = sorted(values)
    if not data:
        return {}
    return {
        k: round(data[math.ceil(len(data) * q) - 1], 3)
        for k, q in (("P50", 0.5), ("P99", 0.99), ("P99.9", 0.999), ("max", 1.0))
    }


def _run(dll: Path, out: Path, uids: dict[int, bytes], hex_sha256: str) -> int:
    nodes_sorted = sorted(uids)
    stats = {
        n: {
            "errors": Counter(),
            "passes": 0,
            "kinds_attempted": Counter(),
            "modes_attempted": Counter(),
            "latencies": [],
        }
        for n in nodes_sorted
    }
    anomalies = Counter()
    start = time.monotonic()
    ordinal = 0

    def elapsed():
        return round((time.monotonic() - start) * 1000, 3)

    def group(bus, writer, nodes, cycle, mode):
        nonlocal ordinal
        pending = {}
        for node in nodes:
            ordinal += 1
            kind = KINDS[(cycle + node - 3) % len(KINDS)]
            sequence = (ordinal + 100) & 65535
            body = struct.pack("<IIHH", 0, ordinal + 100, 0, 0) if kind in (1, 2) else b""
            identifier, data = encode_frames(node, kind, body, sequence)[0]
            sent = time.monotonic()
            writer.append(
                (elapsed(), mode, "tx", node, kind, sequence, hex(identifier), data.hex())
            )
            stats[node]["kinds_attempted"][kind] += 1
            stats[node]["modes_attempted"][mode] += 1
            try:
                bus.send(identifier, data)
                pending[node] = (kind, sequence, sent)
            except OSError as exc:
                stats[node]["errors"]["send"] += 1
                writer.append((elapsed(), mode, "send-error", node, kind, sequence, "", str(exc)))
        while pending:
            deadline = min(t + TIMEOUT for _, _, t in pending.values())
            try:
                item = bus.receive(max(0.0, deadline - time.monotonic()))
            except OSError as exc:
                for node, (kind, sequence, _sent) in pending.items():
                    stats[node]["errors"]["receive/ESI"] += 1
                    writer.append(
                        (elapsed(), mode, "receive-error", node, kind, sequence, "", str(exc))
                    )
                return
            now = time.monotonic()
            for node, (kind, sequence, sent) in list(pending.items()):
                if now - sent > TIMEOUT:
                    stats[node]["errors"]["timeout"] += 1
                    writer.append((elapsed(), mode, "timeout", node, kind, sequence, "", ""))
                    del pending[node]
            if item is None:
                continue
            identifier, data = item
            node = identifier & 255
            fields = struct.unpack_from("<HH", data, 6) if len(data) >= 10 else (-1, -1)
            writer.append((elapsed(), mode, "rx", node, *fields, hex(identifier), data.hex()))
            if (
                identifier != ((3 << 26) | (0xEF << 16) | (2 << 8) | node)
                or node not in pending
                or fields != pending[node][:2]
            ):
                anomalies["unmatched_frame"] += 1
                continue
            kind, sequence, sent = pending.pop(node)
            try:
                payload = decode_reply(identifier, data, node, kind, sequence)
                _validate(uids[node], node, kind, payload)
            except ValueError as exc:
                stats[node]["errors"]["protocol/payload"] += 1
                writer.append((elapsed(), mode, "invalid", node, kind, sequence, "", str(exc)))
            else:
                stats[node]["passes"] += 1
                stats[node]["latencies"].append((now - sent) * 1000)

    fatal = None
    try:
        with ExitStack() as stack:
            stack.enter_context(os.add_dll_directory(str(dll.parent)))
            # 仅调整本次采集进程及其 SDK 线程的调度优先级。
            kernel = C.WinDLL("kernel32", use_last_error=True)
            kernel.GetCurrentProcess.restype = C.c_void_p
            kernel.SetPriorityClass.argtypes = (C.c_void_p, C.c_uint32)
            kernel.GetPriorityClass.argtypes = (C.c_void_p,)
            process = kernel.GetCurrentProcess()
            original_priority = kernel.GetPriorityClass(process)
            if (
                not kernel.SetPriorityClass(process, 0x8000)
                or kernel.GetPriorityClass(process) != 0x8000
            ):
                raise OSError(C.get_last_error(), "set AboveNormal worker priority")
            stack.callback(kernel.SetPriorityClass, process, original_priority)
            bus = stack.enter_context(ControlCanFd(dll, nominal_bps=500_000, data_bps=2_000_000))
            trace = stack.enter_context(
                (out / "frames.csv").open("w", newline="", encoding="utf-8")
            )
            writer = csv.writer(trace)
            writer.writerow(
                (
                    "time_ms",
                    "mode",
                    "event",
                    "node",
                    "kind",
                    "sequence",
                    "can_id",
                    "wire_hex_or_diagnostic",
                )
            )
            for cycle in range(PER_NODE):
                nodes = nodes_sorted[cycle % 5 :] + nodes_sorted[: cycle % 5]
                # 整轮收发结束后才写盘，保留实测版的收发时序。
                events = []
                if (cycle // 100) % 2:
                    group(bus, events, nodes, cycle, "burst5")
                else:
                    for node in nodes:
                        group(bus, events, [node], cycle, "serial")
                writer.writerows(events)
                if (cycle + 1) % 100 == 0:
                    error_count = sum(sum(s["errors"].values()) for s in stats.values())
                    if error_count >= 100:
                        raise RuntimeError(f"diagnostic stop: {error_count} first-attempt failures")
                if (cycle + 1) % 2500 == 0:
                    trace.flush()
                    progress = {
                        "attempted": ordinal,
                        "failures": error_count,
                        "unmatched": sum(anomalies.values()),
                        "elapsed_s": round(elapsed() / 1000, 1),
                    }
                    (out / "progress.json").write_text(json.dumps(progress))
                    print(progress, flush=True)
    except Exception as exc:
        fatal = repr(exc)
    finally:
        report = {
            "worker_priority": "AboveNormal",
            "trace_write": "between rounds",
            "can_nominal_bps": 500000,
            "can_data_bps": 2000000,
            "hex_sha256": hex_sha256,
            "samples_planned_per_node": PER_NODE,
            "timeout_ms": TIMEOUT * 1000,
            "application_retries": 0,
            "duration_s": round(elapsed() / 1000, 3),
            "adapter": dll.name,
            "channel": 0,
            "fatal": fatal,
            "anomalies": anomalies,
            "nodes": {},
        }
        for node, s in stats.items():
            n = sum(s["kinds_attempted"].values())
            errors = sum(s["errors"].values())
            report["nodes"][node] = {
                "attempted": n,
                "passed": s["passes"],
                "errors": s["errors"],
                "kinds": s["kinds_attempted"],
                "modes": s["modes_attempted"],
                "latency_ms": _latency(s["latencies"]),
                "upper_pct_simultaneous95": _upper_pct(n, errors, 0.05 / len(nodes_sorted)),
            }
        errs = sum(sum(s["errors"].values()) for s in stats.values())
        report.update(
            attempted=ordinal,
            failed=errs,
            upper_pct_one_sided95=_upper_pct(ordinal, errs, 0.05),
            latency_ms=_latency([t for s in stats.values() for t in s["latencies"]]),
        )
        report["pass"] = (
            not fatal
            and not anomalies
            and all(
                s["attempted"] == PER_NODE
                and s["passed"] + sum(s["errors"].values()) == PER_NODE
                and s["upper_pct_simultaneous95"] < 0.01
                for s in report["nodes"].values()
            )
        )
        (out / "summary.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)

    return 0 if report["pass"] else 1


def main() -> int:
    """显式选择驱动和批次，在 Windows 上执行五轴只读验收。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dll", required=True, type=Path, help="厂商 x64 ControlCANFD.dll 的路径")
    parser.add_argument(
        "--batch",
        type=Path,
        default=Path(__file__).with_suffix(".json"),
        help="包含 hex_sha256 及五组 node/uid 的烧录批次",
    )
    args = parser.parse_args()
    dll = args.dll.resolve()
    if not dll.is_file():
        parser.error(f"找不到 USB-CAN SDK 文件 {dll}")
    try:
        batch = json.loads(args.batch.read_text(encoding="utf-8-sig"))
        rows = batch["motors"]
        uids = {row["node"]: bytes.fromhex(row["uid"]) for row in rows}
        if (
            len(rows) != 5
            or any(type(node) is not int for node in uids)
            or set(uids) != {3, 4, 5, 6, 7}
            or len(set(uids.values())) != 5
            or any(len(uid) != 12 for uid in uids.values())
            or len(bytes.fromhex(batch["hex_sha256"])) != 32
        ):
            raise ValueError("须包含节点 3 至 7 的五个不同 12 字节 UID 和固件 SHA-256")
    except (OSError, KeyError, TypeError, ValueError) as exc:
        parser.error(f"无法读取烧录批次 {args.batch} ({exc})")
    out = ROOT / "outputs" / time.strftime("can-stability-%Y%m%d-%H%M%S")
    out.mkdir(parents=True, exist_ok=False)
    print(out, flush=True)
    return _run(dll, out, uids, batch["hex_sha256"])


if __name__ == "__main__":
    raise SystemExit(main())
