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
from collections.abc import Callable
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


LEGACY_VERSIONS = (0x602, 0x20200)


def batch_versions(batch: dict) -> tuple[int, int]:
    """历史批次在清单中只记录镜像哈希，当时使用 APP 0.6.2、Loader 2.2.0。"""
    return batch.get("app_version", LEGACY_VERSIONS[0]), batch.get(
        "loader_version", LEGACY_VERSIONS[1]
    )


def _validate(uid: bytes, node: int, kind: int, payload: bytes, versions: tuple[int, int]) -> None:
    if kind == 1 and (
        len(payload) != 32
        or payload[:12] != uid
        or struct.unpack_from("<II", payload, 20) != versions
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


def controlcan_dll_path(resource: Path) -> Path:
    configured = os.environ.get("CONTROLCANFD_DLL")
    dll = Path(configured).resolve() if configured else resource / "ControlCANFD.dll"
    if not dll.is_file():
        raise FileNotFoundError(f"找不到 USB-CAN 驱动库 {dll}")
    return dll


def query_restarted_motor(dll, bus, node, uid, result, stage, *, versions=LEGACY_VERSIONS):
    """查询芯片身份和十次电机状态。每笔独立匹配首答，保留出错前的实测结果。"""
    with os.add_dll_directory(str(dll.parent)), ControlCanFd(dll, **bus) as port:
        for seq in range(1, 12):
            kind = 1 if seq == 1 else 108
            label = "芯片身份及固件版本" if seq == 1 else f"电机状态 {seq - 1}/10"
            stage(f"节点 {node} 查询{label}")
            body = struct.pack("<IIHH", 0, seq, 0, 0) if kind == 1 else b""
            ((tx_id, request),) = encode_frames(node, kind, body, seq)
            row = {"kind": kind, "sequence": seq, "tx_id": hex(tx_id), "passed": False}
            result["queries"].append(row)
            started = time.monotonic()
            port.send(tx_id, request)
            deadline = started + TIMEOUT
            while (remaining := deadline - time.monotonic()) > 0:
                reply = port.receive(remaining)
                elapsed = time.monotonic() - started
                if reply is None or elapsed > TIMEOUT:
                    break
                rx_id, data = reply
                if rx_id != (0x0CEF0200 | node):
                    continue
                if len(data) >= 10 and struct.unpack_from("<HH", data, 6) != (kind, seq):
                    continue
                payload = decode_reply(rx_id, data, node, kind, seq)
                _validate(uid, node, kind, payload, versions)
                row.update(
                    rx_id=hex(rx_id),
                    reply_hex=data.hex(),
                    latency_ms=round(elapsed * 1000, 3),
                    passed=True,
                )
                break
            if not row["passed"]:
                raise TimeoutError(f"节点 {node} 的{label}在 50 ms 内未收到匹配应答")
            result["passed"] += 1
            detail = f"UID {uid.hex()}，固件版本已核对　" if kind == 1 else ""
            stage(f"{label}通过　{detail}应答 {row['rx_id']}，{row['latency_ms']:.3f} ms")


def _latency(values: list[float]) -> dict[str, float]:
    data = sorted(values)
    if not data:
        return {}
    return {
        k: round(data[math.ceil(len(data) * q) - 1], 3)
        for k, q in (("P50", 0.5), ("P99", 0.99), ("P99.9", 0.999), ("max", 1.0))
    }


LADDER = ((5, 20), (50, 30), (200, 30), (500, 60), (5, 20))


def run_queries(
    dll: Path,
    out: Path,
    uids: dict[int, bytes],
    hex_sha256: str,
    *,
    versions: tuple[int, int] = LEGACY_VERSIONS,
    profile: str = "acceptance",
    per_node: int = PER_NODE,
    rate_hz: int = 50,
    channel_index: int = 0,
    nominal_bps: int = 1_000_000,
    data_bps: int = 5_000_000,
    stopped: Callable[[], bool] | None = None,
    on_progress: Callable[[dict], None] | None = None,
) -> int:
    """CLI 与桌面端共用的只读收发。停止在采集进程释放适配器后生效。"""
    if not uids or profile not in ("fixed", "ladder", "acceptance") or min(per_node, rate_hz) <= 0:
        raise ValueError("节点、模式或采样参数无效")
    if profile == "acceptance" and (
        set(uids) != {3, 4, 5, 6, 7}
        or per_node != PER_NODE
        or (nominal_bps, data_bps) not in ((500_000, 2_000_000), (1_000_000, 5_000_000))
    ):
        raise ValueError("固定验收需要五个登记节点、1M/5M 或 500K/2M 和每节点 70000 笔")
    total = (
        sum(hz * seconds for hz, seconds in LADDER) if profile == "ladder" else len(uids) * per_node
    )
    cancelled = False
    stages = []

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
    rx_flags = Counter()
    start = time.monotonic()
    ordinal = 0

    def elapsed():
        return round((time.monotonic() - start) * 1000, 3)

    def group(bus, writer, nodes, cycle, mode):
        nonlocal ordinal
        pending = {}
        flagged = set()
        for node in nodes:
            ordinal += 1
            kind = KINDS[(cycle + node - 3) % len(KINDS)] if profile == "acceptance" else 108
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
            frames = bus.read_fd_batch()
            now = time.monotonic()
            for node, (kind, sequence, sent) in list(pending.items()):
                if now - sent > TIMEOUT:
                    reason = "flagged_only" if node in flagged else "timeout"
                    stats[node]["errors"][reason] += 1
                    writer.append((elapsed(), mode, reason, node, kind, sequence, "", ""))
                    del pending[node]
            for item in frames:
                frame = item.frame
                identifier = frame.can_id
                node = identifier & 255
                data = bytes(frame.data[: frame.length])
                fields = struct.unpack_from("<HH", data, 6) if len(data) >= 10 else (-1, -1)
                event = "rx"
                if frame.err or frame.esi:
                    event = (
                        "rx_err_esi"
                        if frame.err and frame.esi
                        else ("rx_err" if frame.err else "rx_esi")
                    )
                    rx_flags[event] += 1
                writer.append((elapsed(), mode, event, node, *fields, hex(identifier), data.hex()))
                if (
                    not frame.eff
                    or not frame.brs
                    or frame.rtr
                    or frame.length > 64
                    or identifier != ((3 << 26) | (0xEF << 16) | (2 << 8) | node)
                    or node not in pending
                    or fields != pending[node][:2]
                ):
                    anomalies["unmatched_frame"] += 1
                    continue
                kind, sequence, sent = pending[node]
                try:
                    payload = decode_reply(identifier, data, node, kind, sequence)
                    _validate(uids[node], node, kind, payload, versions)
                except ValueError as exc:
                    if event == "rx":
                        stats[node]["errors"]["protocol/payload"] += 1
                        del pending[node]
                    else:
                        anomalies["invalid_flagged_frame"] += 1
                    writer.append((elapsed(), mode, "invalid", node, kind, sequence, "", str(exc)))
                    continue
                if event != "rx":
                    # 继续等待同一请求的正常回复，其余在途请求独立计时。
                    flagged.add(node)
                    continue
                stats[node]["passes"] += 1
                stats[node]["latencies"].append((now - sent) * 1000)
                del pending[node]
            if pending and not frames:
                time.sleep(0.0005)

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
            bus = stack.enter_context(
                ControlCanFd(
                    dll, channel_index=channel_index, nominal_bps=nominal_bps, data_bps=data_bps
                )
            )
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
            last_update = start

            def progress(stage, *, force=False):
                nonlocal last_update
                now = time.monotonic()
                if not force and now - last_update < 0.3:
                    return
                last_update = now
                message = {
                    "attempted": ordinal,
                    "planned": total,
                    "stage": stage,
                    "failures": sum(sum(s["errors"].values()) for s in stats.values()),
                    "unmatched": sum(anomalies.values()),
                    "elapsed_s": round(now - start, 1),
                }
                if on_progress:
                    on_progress(message)
                if force:
                    trace.flush()
                    (out / "progress.json").write_text(json.dumps(message), encoding="utf-8")

            if profile == "acceptance":
                for cycle in range(per_node):
                    if stopped and stopped():
                        cancelled = True
                        break
                    nodes = nodes_sorted[cycle % 5 :] + nodes_sorted[: cycle % 5]
                    events = []
                    mode = "burst5" if (cycle // 100) % 2 else "serial"
                    if mode == "burst5":
                        group(bus, events, nodes, cycle, mode)
                    else:
                        for node in nodes:
                            group(bus, events, [node], cycle, mode)
                    writer.writerows(events)
                    if (cycle + 1) % 100 == 0 and sum(
                        sum(s["errors"].values()) for s in stats.values()
                    ) >= 100:
                        raise RuntimeError("累计首次失败达到 100 笔，结束诊断")
                    progress(mode, force=(cycle + 1) % 2500 == 0)
            else:
                schedule = (
                    [(hz, hz * seconds) for hz, seconds in LADDER]
                    if profile == "ladder"
                    else [(rate_hz, total)]
                )
                for hz, count in schedule:
                    begin = time.monotonic()
                    sent_before = ordinal
                    passed_before = sum(s["passes"] for s in stats.values())
                    next_send = begin
                    label = f"{hz} 笔/秒"
                    for _ in range(count):
                        if stopped and stopped():
                            cancelled = True
                            break
                        wait = next_send - time.monotonic()
                        if wait > 0:
                            time.sleep(wait)
                        if stopped and stopped():
                            cancelled = True
                            break
                        next_send = max(next_send + 1 / hz, time.monotonic())
                        events = []
                        group(bus, events, [nodes_sorted[ordinal % len(nodes_sorted)]], 0, profile)
                        writer.writerows(events)
                        progress(label)
                    duration = time.monotonic() - begin
                    attempts = ordinal - sent_before
                    stages.append(
                        {
                            "target_hz": hz,
                            "attempted": attempts,
                            "passed": sum(s["passes"] for s in stats.values()) - passed_before,
                            "duration_s": round(duration, 3),
                            "actual_hz": round(attempts / duration, 2) if duration else 0,
                        }
                    )
                    if cancelled:
                        break
            progress(profile, force=True)
    except Exception as exc:
        fatal = repr(exc)
    finally:
        report = {
            "worker_priority": "AboveNormal",
            "trace_write": "between rounds",
            "can_nominal_bps": nominal_bps,
            "can_data_bps": data_bps,
            "hex_sha256": hex_sha256,
            "samples_planned_per_node": per_node if profile != "ladder" else None,
            "timeout_ms": TIMEOUT * 1000,
            "application_retries": 0,
            "duration_s": round(elapsed() / 1000, 3),
            "adapter": dll.name,
            "channel": channel_index,
            "profile": profile,
            "stages": stages,
            "cancelled": cancelled,
            "planned": total,
            "transmit_type": 0,
            "fatal": fatal,
            "anomalies": anomalies,
            "rx_flags": rx_flags,
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
        report["unresolved"] = ordinal - errs - sum(s["passes"] for s in stats.values())
        report["complete"] = not fatal and not cancelled and ordinal == total
        report["pass"] = (
            report["complete"]
            and not anomalies
            and not rx_flags
            and all(
                s["attempted"] == per_node
                and s["passed"] + sum(s["errors"].values()) == per_node
                and s["upper_pct_simultaneous95"] < 0.01
                for s in report["nodes"].values()
            )
        )
        if profile != "acceptance":
            report["pass"] = report["complete"] and not errs and not anomalies and not rx_flags
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
    return run_queries(
        dll,
        out,
        uids,
        batch["hex_sha256"],
        versions=batch_versions(batch),
        nominal_bps=batch.get("can_arbitration_bps", 500_000),
        data_bps=batch.get("can_data_bps", 2_000_000),
    )


if __name__ == "__main__":
    raise SystemExit(main())
